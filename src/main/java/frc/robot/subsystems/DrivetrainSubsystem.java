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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.FFGains;
import frc.robot.util.NeoSwerveModule;
import frc.robot.util.PIDGains;
import frc.robot.util.SwerveModule;
import frc.robot.util.Util;
import frc.robot.LimelightHelpers;

import static frc.robot.Constants.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;

public class DrivetrainSubsystem implements Subsystem {

    /*
     * Robot Coordinate System
     * See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
     * Forward is x+, Left is y+, counterclockwise is theta+
     */

    // Limelight
    private static boolean useVision = false;

    private final NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber validTargetSubscriber = limelightNT.getDoubleTopic("tv").subscribe(0);
    private final DoubleArraySubscriber botPoseBlueSubscriber = limelightNT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);

    double tx = LimelightHelpers.getTX("limelight");
    double ty = LimelightHelpers.getTY("limelight");
   
    double KpAim = -0.1f;
    double KpDistance = 0.1f;
    double min_aim_command = 0.05f;

    float left_command;
    float right_command;
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

    private final StructPublisher<Rotation2d> adjustedRotationPublisher = drivetrainNT.getStructTopic(
        "adjustedRotation",
        Rotation2d.struct
    ).publish();

    private final StructPublisher<Rotation2d> rotationPublisher = drivetrainNT.getStructTopic(
        "rotation",
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
    private final ProfiledPIDController rotationController = new ProfiledPIDController(3, 1.0e-5, 1.0e-1, new TrapezoidProfile.Constraints(ANGULAR_VELOCITY_CONSTRAINT, ANGULAR_ACCEL_CONSTRAINT));
    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        rotationController);
    
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
            ON_RED_ALLIANCE,
            this
        );

        // Drive FFGain updated AM 03/07

        frontLeft = new NeoSwerveModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_STEER_MOTOR_ID,
                FONT_LEFT_STEER_ENCODER_ID,
                FRONT_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.19861, 3.2379, 0.562),
                drivetrainNT.getSubTable("frontleft"));

        frontRight = new NeoSwerveModule(
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_STEER_ENCODER_ID,
                FRONT_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.18406, 3.2722, 0.40914),
                drivetrainNT.getSubTable("frontright"));

        backLeft = new NeoSwerveModule(
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_STEER_ENCODER_ID,
                BACK_LEFT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.17395, 3.286, 0.51328),
                drivetrainNT.getSubTable("backleft"));

        backRight = new NeoSwerveModule(
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_STEER_ENCODER_ID,
                BACK_RIGHT_STEER_OFFSET,
                new PIDGains(1.0, 0, 0),
                new PIDGains(1, 1.0e-6, 0),
                new FFGains(0.17731, 3.2446, 0.41604),
                drivetrainNT.getSubTable("backright"));

        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics,
                getRotation(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }, 
                new Pose2d());

        resetPose(new Pose2d(new Translation2d(10, 0), Rotation2d.fromDegrees(0)));
        //zeroGyro();
    }


    @Override
    public void periodic() {
        // does not need to use adjusted rotation, odometry handles it.
        //updates pose with rotation and swerve positions
        poseEstimator.update(getRotation(), getSwervePositions());

        if(useVision)
            visionPosePeriodic();

        updateTelemetry();

        frontLeft.periodic();
        frontRight.periodic();
        backLeft.periodic();
        backRight.periodic();
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
        }, RobotController.getFPGATime() - 200000);

        adjustedRotationPublisher.set(getAdjustedRotation());
        rotationPublisher.set(getRotation());

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
        
        //Returns if the botpose doesn't give an array with 7 variables or more
        double[] botPose = botPoseBlueSubscriber.get();
        if(botPose.length < 7)
            return;
        
        // Convert double[] from NT to Pose2D
        Pose2d visionPose = new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
        double measurementTime = Timer.getFPGATimestamp() - botPose[6] / 1000; // calculate the actual time the picture was taken

        poseEstimator.addVisionMeasurement(visionPose, measurementTime);
    }

    private void AimAssist(){

        double heading_error = -tx;
        double distance_error = -ty;
        double steering_adjust = 0.0f;

        if(tx > 1.0)
        {
            steering_adjust = KpAim * heading_error - min_aim_command;
        } else if(tx < -1.0){
            steering_adjust = KpAim*heading_error + min_aim_command;
        }

        double distance_adjust = KpDistance * distance_error;

        left_command += steering_adjust + distance_adjust;
        right_command -= steering_adjust + distance_adjust;
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
        rotationOffsetRadians = -getRotation().getRadians();
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

    /**
     * @return a rotation2d object representing the robot's zeored heading, with 0 degrees being the direction the robot will drive forward in
     */
    public Rotation2d getAdjustedRotation() {
        return getRotation().plus(Rotation2d.fromRadians(rotationOffsetRadians));
    }

    /**
     * @return a rotation2d object representing the robot's current heading, with 0 degrees being the direction the robot was facing at startup
     */
    public Rotation2d getRotation() {
        return navX.getRotation2d().plus(Rotation2d.fromRadians(Math.PI));
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
        poseEstimator.resetPosition(getRotation(), getSwervePositions(), pose);
    }

    public void setGyro(Rotation2d rotation) {
        rotationOffsetRadians = -getRotation().getRadians() + rotation.getRadians();
    }


    /**
     * Converts raw joystick values to speeds for the drivetrain
     * <p>
     * This method applies deadbands and curves to the joystick values and clamps the resultant speed to the linear velocity constraint
     * 
     * @param xAxis value from -1 to 1 representing the x-axis of the joystick
     * @param yAxis value from -1 to 1 representing the y-axis of the joystick
     * @param rotAxis value from -1 to 1 representing the rotation axis of the joystick
     * @return a ChassisSpeeds object representing the speeds to be passed to the drivetrain
     */
    public ChassisSpeeds joystickAxesToChassisSpeeds(double xAxis, double yAxis, double rotAxis) {

        double xVelocity = Util.translationCurve(MathUtil.applyDeadband(xAxis, XBOX_DEADBAND)) * LINEAR_VELOCITY_CONSTRAINT * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
        double yVelocity = Util.translationCurve(MathUtil.applyDeadband(yAxis, XBOX_DEADBAND)) * LINEAR_VELOCITY_CONSTRAINT * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
        double rotVelocity = Util.steerCurve(MathUtil.applyDeadband(rotAxis, XBOX_DEADBAND)) * ANGULAR_VELOCITY_CONSTRAINT * (slowMode ? ANGULAR_SLOW_MODE_MODIFIER : 1);
        
        double totalVelocity = Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2));

        if (totalVelocity > LINEAR_VELOCITY_CONSTRAINT){
            xVelocity *= (LINEAR_VELOCITY_CONSTRAINT / totalVelocity);
            yVelocity *= (LINEAR_VELOCITY_CONSTRAINT / totalVelocity);
        }

        return new ChassisSpeeds(xVelocity, yVelocity, rotVelocity);
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
        return Commands.run(() -> {

            ChassisSpeeds speeds = joystickAxesToChassisSpeeds(xAxis.getAsDouble(), yAxis.getAsDouble(), rotAxis.getAsDouble());
            drive(speeds, true, true);

        }, this);
    }

    // SysID Routine Commands
    public Command driveSysIdRoutineCommand() {
        return Commands.sequence(
            driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(7),
            Commands.waitSeconds(2),
            driveSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(7),
            Commands.waitSeconds(2),
            driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(2),
            Commands.waitSeconds(2),
            driveSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(2),
            Commands.waitSeconds(2)
        );
    }

    public Command steerSysIdRoutineCommand() {
        return Commands.sequence(
            steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withTimeout(7),
            Commands.waitSeconds(2),
            steerSysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).withTimeout(7),
            Commands.waitSeconds(2),
            steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).withTimeout(2),
            Commands.waitSeconds(2),
            steerSysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).withTimeout(2),
            Commands.waitSeconds(2)
        );
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

    public Command driveAndPointToSpeakerCommand(DoubleSupplier xAxis, DoubleSupplier yAxis) {
        return pointToSpeakerWithSpeedsCommand(() -> (joystickAxesToChassisSpeeds(xAxis.getAsDouble(), yAxis.getAsDouble(), 0)));
    }

    public Command pointToSpeakerCommand() {
        return pointToSpeakerWithSpeedsCommand(() -> (new ChassisSpeeds()));
    }

    public Command pointToSpeakerWithSpeedsCommand(Supplier<ChassisSpeeds> speedsSupplier) {
        return Commands.run(() -> {

            double xDistance = poseEstimator.getEstimatedPosition().getX() - 0.0f;
            double yDistance = poseEstimator.getEstimatedPosition().getY() - 5.2f;

            // TODO: Account for alliance
            //improve this math eventually
            //assuming Blue Speaker, and Blue is Left/Red is Right and speakers/amps are on the top half of the arena
            double desiredAngle = Math.atan(yDistance/xDistance);
            
            drive(speedsSupplier.get().vxMetersPerSecond, speedsSupplier.get().vyMetersPerSecond, rotationController.calculate(getPose().getRotation().getRadians(), desiredAngle), true, true);

        }, this);
    }
    
}
