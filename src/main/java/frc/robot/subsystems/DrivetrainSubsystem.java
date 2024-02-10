package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.FFGains;
import frc.robot.util.NeoSwerveModule;
import frc.robot.util.PIDGains;
import frc.robot.util.SwerveModule;
import frc.robot.util.Util;

import static frc.robot.Constants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

public class DrivetrainSubsystem implements Subsystem {

    /*
     * Robot Coordinate System
     * See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
     * Forward is x+, Left is y+, counterclockwise is theta+
     */

    // Limelight
    private static boolean useVision = true;

    private final NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber validTargetSubscriber = limelightNT.getDoubleTopic("tv").subscribe(0);
    private final DoubleArraySubscriber botPoseBlueSubscriber = limelightNT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);

    // Telemetry
    public static final NetworkTable drivetrainNT = NetworkTableInstance.getDefault().getTable("drivetrain");
    
    private final StructArrayPublisher<SwerveModuleState> measuredSwerveStatesPublisher = drivetrainNT.getStructArrayTopic(
        "measuredSwerveStates",
        SwerveModuleState.struct
        ).publish();

    private final StructArrayPublisher<SwerveModuleState> desiredSwerveStatesPublisher = drivetrainNT.getStructArrayTopic(
        "desiredSwerveStates",
        SwerveModuleState.struct
        ).publish();
    
        private final StructPublisher<Pose2d> robotPosePublisher = drivetrainNT.getStructTopic("robotPose", Pose2d.struct).publish();

        private final StructPublisher<Rotation2d> robotRotationPublisher = drivetrainNT.getStructTopic(
            "robotRotation",
            Rotation2d.struct
        ).publish();

    // SysID
    private final SysIdRoutine driveSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage) -> runDriveVolts(voltage.in(Units.Volts)),
            null,
            this
        )
    );

    private final SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            (voltage) -> runSteerVolts(voltage.in(Units.Volts)),
            null,
            this
        )
    );

    // Swerve
    private final Translation2d frontLeftPosition = new Translation2d(DRIVETRAIN_WIDTH / 2D, DRIVETRAIN_LENGTH / 2D); // All translations are relative to center of rotation
    private final Translation2d frontRightPosition = new Translation2d(DRIVETRAIN_WIDTH / 2D, -DRIVETRAIN_LENGTH / 2D);
    private final Translation2d backLeftPosition = new Translation2d(-DRIVETRAIN_WIDTH / 2D, DRIVETRAIN_LENGTH / 2D);
    private final Translation2d backRightPosition = new Translation2d(-DRIVETRAIN_WIDTH / 2D, -DRIVETRAIN_LENGTH / 2D);

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
    private final SwerveDrivePoseEstimator poseEstimator;
    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(ANGULAR_VELOCITY_CONSTRAINT, ANGULAR_ACCEL_CONSTRAINT)));
    
    private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    private ChassisSpeeds currentChassisSpeeds = new ChassisSpeeds();
    private boolean slowMode = false;
    private double rotationOffsetRadians = 0.0;

    private ChassisSpeeds lastCommandedChassisSpeeds = new ChassisSpeeds();

    public DrivetrainSubsystem() {

        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetPose,
            this::getChassisSpeeds,
            this::robotRelativeDrive,
            HOLONOMIC_PATH_FOLLOWER_CONFIG,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeft = new NeoSwerveModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_STEER_ENCODER_ID,
                FRONT_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(2*0, 0, 0),
                new FFGains(0.15263, 3.158, 0.53993),
                drivetrainNT.getSubTable("frontleft"));
        
        frontRight = new NeoSwerveModule(
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_STEER_ENCODER_ID,
                FRONT_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1*0, 0, 0),
                new FFGains(0.17645, 3.1584, 0.30427),
                drivetrainNT.getSubTable("frontright"));

        backLeft = new NeoSwerveModule(
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_STEER_ENCODER_ID,
                BACK_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(0.45027 * 0, 0, 0),
                new FFGains(0.1464, 3.206, 0.44254),
                drivetrainNT.getSubTable("backleft"));
        
        backRight = new NeoSwerveModule(
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_STEER_ENCODER_ID,
                BACK_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(2 * 0, 0, 0),
                new FFGains(0.1751, 3.1887, 0.31847),
                drivetrainNT.getSubTable("backright"));

        tab.addNumber("Rotation", () -> (getAdjustedRotation().getDegrees()));

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }, 
                new Pose2d());

        resetPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    }


    @Override
    public void periodic() {
        // does not need to use adjusted rotation, odometry handles it.
        //updates pose with rotation and swerve positions
        poseEstimator.update(navX.getRotation2d(), getSwervePositions());

        if(useVision)
            visionPosePeriodic();

        updateTelemetry();
    }

    public void robotRelativeDrive(ChassisSpeeds chassisSpeeds) {
        drive(chassisSpeeds, false, false);
    }

    private void updateTelemetry(){
        //Swerve
        desiredSwerveStatesPublisher.set(new SwerveModuleState[] {
            frontLeft.getDesiredState(),
            frontRight.getDesiredState(),
            backLeft.getDesiredState(),
            backRight.getDesiredState()
        });

        measuredSwerveStatesPublisher.set(new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });

        robotRotationPublisher.set(getAdjustedRotation());

        frontLeft.updateTelemetry();
        frontRight.updateTelemetry();
        backLeft.updateTelemetry();
        backRight.updateTelemetry();

        robotPosePublisher.set(getPose());
    }

    // tracks position with vision
    private void visionPosePeriodic(){

        // Return if the limelight doesn't see a target
        if(validTargetSubscriber.get() != 1)
            return;
        
        double[] botPose = botPoseBlueSubscriber.get();
        if(botPose.length < 7)
            return;
        
        // Convert double[] from NT to Pose2D
        Pose2d visionPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
        double measurementTime = Timer.getFPGATimestamp() - botPose[6] / 1000; // calculate the actual time the picture was taken

        poseEstimator.addVisionMeasurement(visionPose, measurementTime);
    }

    public void drive(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative, boolean limitAcceleration) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angularVelocity);
        if(limitAcceleration)
            desaturateChassisSpeedsAcceleration(chassisSpeeds);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getAdjustedRotation()) : chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_ATTAINABLE_VELOCITY);
        setModuleStates(moduleStates);

        lastCommandedChassisSpeeds = chassisSpeeds;
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative, boolean limitAcceleration) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative, limitAcceleration);
    }

    private long lastTime = 0;

    private void desaturateChassisSpeedsAcceleration(ChassisSpeeds speeds) {
        long currentTime = RobotController.getFPGATime();
        double dtSeconds = (currentTime - lastTime) / 1e6;
        lastTime = currentTime;

        double accelX = (speeds.vxMetersPerSecond - lastCommandedChassisSpeeds.vxMetersPerSecond) / dtSeconds;
        double accelY = (speeds.vyMetersPerSecond - lastCommandedChassisSpeeds.vyMetersPerSecond) / dtSeconds;
        double linearAccelMag = Math.sqrt(accelX * accelX + accelY * accelY);

        // No need to limit decceleration ie. return if acceleration is in the opposite direction as current travel
        if(accelX * lastCommandedChassisSpeeds.vxMetersPerSecond < 0 && accelY * lastCommandedChassisSpeeds.vyMetersPerSecond < 0)
            return;

        if(linearAccelMag > LINEAR_ACCEL_CONSTRAINT) {
            accelX *= Math.abs(LINEAR_ACCEL_CONSTRAINT / linearAccelMag);
            accelY *= Math.abs(LINEAR_ACCEL_CONSTRAINT / linearAccelMag);

            speeds.vxMetersPerSecond = lastCommandedChassisSpeeds.vxMetersPerSecond + (accelX * dtSeconds);
            speeds.vyMetersPerSecond = lastCommandedChassisSpeeds.vyMetersPerSecond + (accelY * dtSeconds);
        }

        double angularAccel = (speeds.omegaRadiansPerSecond - lastCommandedChassisSpeeds.omegaRadiansPerSecond) / dtSeconds;

        if(Math.abs(angularAccel) > ANGULAR_ACCEL_CONSTRAINT) {
            angularAccel = Math.signum(angularAccel) * ANGULAR_ACCEL_CONSTRAINT;
            speeds.omegaRadiansPerSecond = lastCommandedChassisSpeeds.omegaRadiansPerSecond + (angularAccel * dtSeconds);
        }        
    }

    public void stopMotion() {
        drive(0, 0, 0, false, false);
    }

    public void zeroGyro() {
        rotationOffsetRadians = -navX.getRotation2d().getRadians();
    }

    public void runDriveVolts(double voltage) {
        frontLeft.runForward(voltage);
        frontRight.runForward(voltage);
        backLeft.runForward(voltage);
        backRight.runForward(voltage);
    }

    public void runSteerVolts(double voltage) {
        frontLeft.runRotation(voltage);
        frontRight.runRotation(voltage);
        backLeft.runRotation(voltage);
        backRight.runRotation(voltage);
    }

    // Getters

    public SwerveModulePosition[] getSwervePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public AHRS getNavx() {
        return navX;
    }

    public HolonomicDriveController getDriveController() {
        return driveController;
    }

    public Rotation2d getAdjustedRotation() {
        return navX.getRotation2d().plus(Rotation2d.fromRadians(rotationOffsetRadians));
    }

    public ChassisSpeeds getChassisSpeeds() {
        return currentChassisSpeeds;
    }


    // Setters

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(navX.getRotation2d(), getSwervePositions(), pose);
    }

    public void setGyro(Rotation2d rotation) {
        rotationOffsetRadians = -navX.getRotation2d().getRadians() + rotation.getRadians();
    }

    // Commands

    public Command enableSlowModeCommand() {
        return Commands.runOnce(() -> { slowMode = true; });
    }

    public Command disableSlowModeCommand() {
        return Commands.runOnce(() -> { slowMode = false; });
    }

    public Command zeroGyroCommand() {
        return Commands.runOnce(this::zeroGyro, this);
    }

    public Command joystickDriveCommand(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotAxis) {
        double deadband = 0.06;
        return Commands.run(() -> {

            double xVelocity = Util.translationCurve(MathUtil.applyDeadband(xAxis.getAsDouble(), deadband)) * LINEAR_VELOCITY_CONSTRAINT * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double yVelocity = Util.translationCurve(MathUtil.applyDeadband(yAxis.getAsDouble(), deadband)) * LINEAR_VELOCITY_CONSTRAINT * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double rotVelocity = Util.steerCurve(MathUtil.applyDeadband(rotAxis.getAsDouble(), deadband)) * ANGULAR_VELOCITY_CONSTRAINT * (slowMode ? ANGULAR_SLOW_MODE_MODIFIER : 1);
            
            double totalVelocity = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));

            if (totalVelocity > LINEAR_VELOCITY_CONSTRAINT){
                xVelocity *= (LINEAR_VELOCITY_CONSTRAINT / totalVelocity);
                yVelocity *= (LINEAR_VELOCITY_CONSTRAINT / totalVelocity);
            }

            drive(xVelocity, yVelocity, rotVelocity, FIELD_RELATIVE_DRIVE, true);

        }, this);
    }

    // SysID Routine Commands
    public Command driveSysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.quasistatic(direction);
    }

    public Command steerSysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return steerSysIdRoutine.quasistatic(direction);
    }

    public Command driveSysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return driveSysIdRoutine.dynamic(direction);
    }

    public Command steerSysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return steerSysIdRoutine.dynamic(direction);
    }
    
    public Command followPathCommand(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        return new FollowPathHolonomic(
            path, 
            this::getPose,
            this::getChassisSpeeds,
            this::robotRelativeDrive,
            HOLONOMIC_PATH_FOLLOWER_CONFIG,
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this
        );    
    }

    public void pointToSpeaker(){
        double xDistance = poseEstimator.getEstimatedPosition().getX() - 0.2f;
        double yDistance = poseEstimator.getEstimatedPosition().getY() - 5.55f;

        //improve this math eventually
        double actualAngle = Math.signum(yDistance) //above speaker means +1, below means -1
        * (((Math.signum(xDistance) - 1) * -90) //in front of speaker (right) means 180, behind (left) means 0
        - (Math.signum(xDistance) * (Math.atan(Math.abs(yDistance)/Math.abs(xDistance))))); //in front means neg, behind means pos


        //beyond this point is "idk how this works but im guessing this is how"

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        poseEstimator.getEstimatedPosition(),
        null,
        new Pose2d(poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY(), Rotation2d.fromDegrees(actualAngle)),
        new TrajectoryConfig(LINEAR_VELOCITY_CONSTRAINT, LINEAR_ACCEL_CONSTRAINT));

        //insert amount needing to rotate/max angular speed... probably maybe idk
        Trajectory.State goal = trajectory.sample(2);

        ChassisSpeeds adjustedSpeeds = driveController.calculate(
        poseEstimator.getEstimatedPosition(), goal, Rotation2d.fromDegrees(actualAngle));

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(adjustedSpeeds);

        setModuleStates(moduleStates);
    }
}
