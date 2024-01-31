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
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.NeoSwerveModule;
import frc.robot.util.SwerveModule;

import static frc.robot.Constants.*;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

public class DrivetrainSubsystem implements Subsystem {

    /*
     * Robot Coordinate System
     * See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/geometry/coordinate-systems.html#robot-coordinate-system
     * Forward is x+, Left is y+, counterclockwise is theta+
     */

    //limelight definitions
    private static boolean useVision = true;

    
    private final NetworkTable limelightNT = NetworkTableInstance.getDefault().getTable("limelight");
    private final DoubleSubscriber validTargetSubscriber = limelightNT.getDoubleTopic("tv").subscribe(0);
    private final DoubleArraySubscriber botPoseBlueSubscriber = limelightNT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[0]);

    // Telemetry

    public static final NetworkTable drivetrainNT = NetworkTableInstance.getDefault().getTable("drivetrain");
    
    private final StructArrayPublisher<SwerveModuleState> measuredSwerveStatesPublisher = drivetrainNT.getStructArrayTopic(
        "MeasuredSwerveStates",
        SwerveModuleState.struct
        ).publish();

    private final StructArrayPublisher<SwerveModuleState> desiredSwerveStatesPublisher = drivetrainNT.getStructArrayTopic(
        "DesiredSwerveStates",
        SwerveModuleState.struct
        ).publish();
    
        private final StructPublisher<Pose2d> robotPosePublisher = drivetrainNT.getStructTopic("RobotPose", Pose2d.struct).publish();

        private final StructPublisher<Rotation2d> robotRotationPublisher = drivetrainNT.getStructTopic(
            "RobotRotation",
            Rotation2d.struct
        ).publish();


    
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
   // private final SwerveDriveOdometry odometry;
    private final Field2d field;

    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL)));
    
    private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    private boolean slowMode = false;
    private double rotationOffsetRadians = 0.0;

    public DrivetrainSubsystem(Field2d field) {
        this.field = field;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeft = new NeoSwerveModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_STEER_ENCODER_ID,
                FRONT_LEFT_STEER_OFFSET,
                drivetrainNT.getSubTable("frontleft"));
        
        frontRight = new NeoSwerveModule(
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_STEER_ENCODER_ID,
                FRONT_RIGHT_STEER_OFFSET,
                drivetrainNT.getSubTable("frontright"));

        backLeft = new NeoSwerveModule(
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_STEER_ENCODER_ID,
                BACK_LEFT_STEER_OFFSET,
                drivetrainNT.getSubTable("backleft"));
        
        backRight = new NeoSwerveModule(
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_STEER_ENCODER_ID,
                BACK_RIGHT_STEER_OFFSET,
                drivetrainNT.getSubTable("backright"));

        tab.add("Test Drivetrain", testDrivetrain()).withPosition(8, 0);

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
        field.setRobotPose(getPose());


        if(useVision)
            visionPosePeriodic();

        updateTelemetry();
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
    //tracks position with vision
    private void visionPosePeriodic(){

        if(validTargetSubscriber.get() != 1)
            return;
        
        double[] botPose = botPoseBlueSubscriber.get();
        if(botPose.length < 7)
            return;
        
        Pose2d visionPose =  new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
        double measurementTime= Timer.getFPGATimestamp() - botPose[6] / 1000;

        poseEstimator.addVisionMeasurement(visionPose, measurementTime);
    }

    public void drive(double xSpeed, double ySpeed, double angularVelocity, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, angularVelocity);

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(fieldRelative? ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getAdjustedRotation()) : chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_LINEAR_VELOCITY);

        setModuleStates(moduleStates);
    }

    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, fieldRelative);
    }

    public void stopMotion() {
        drive(0, 0, 0, false);
    }

    public void zeroGyro() {
        rotationOffsetRadians = -navX.getRotation2d().getRadians();
        rotationOffsetRadians = -navX.getRotation2d().getRadians();
        //resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
        rotationOffsetRadians = -navX.getRotation2d().getRadians();   
        //resetPose(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(0)));
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

    // Setters

    public void setModuleStates(SwerveModuleState[] moduleStates) {
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    public void resetPose(Pose2d pose) {
        //resets pose
        poseEstimator.resetPosition(navX.getRotation2d(), getSwervePositions(), pose);
    }

    public void setGyro(Rotation2d rotation) {
        rotationOffsetRadians = -navX.getRotation2d().getRadians() + rotation.getRadians();
    }

    // Commands

    public Command testDrivetrain() {
        return Commands.sequence(
            Commands.run(() -> {
                setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(0))
                });
            }, this).withTimeout(1),
            Commands.run(() -> {
                setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(0))
                });
            }, this).withTimeout(1),
            Commands.run(() -> {
                setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(90)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(90)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(90)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(90))
                });
            }, this).withTimeout(1),
            Commands.run(() -> {
                setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(180)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(180)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(180)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(180))
                });
            }, this).withTimeout(1),
            Commands.run(() -> {
                setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(-90)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(-90)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(-90)),
                    new SwerveModuleState(1.0, Rotation2d.fromDegrees(-90))
                });
            }, this).withTimeout(1),
            Commands.run(() -> {
                setModuleStates(new SwerveModuleState[] {
                    new SwerveModuleState(0, Rotation2d.fromDegrees(360)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(360)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(360)),
                    new SwerveModuleState(0, Rotation2d.fromDegrees(360))
                });
            }, this).withTimeout(1),
            Commands.run(() -> {
                drive(0, 0.0, 0, false);
            }, this).withTimeout(1),
            Commands.run(() -> {
                drive(1.0, 0.0, 0, false);
            }, this).withTimeout(1),
            Commands.run(() -> {
                drive(-1.0, 0.0, 0, false);
            }, this).withTimeout(1),
            Commands.run(() -> {
                drive(0.0, 1.0, 0, false);
            }, this).withTimeout(1),
            Commands.run(() -> {
                drive(0.0, -1.0, 0, false);
            }, this).withTimeout(1),
            Commands.run(() -> {
                drive(0.0, 0.0, 1.0, false);
            }, this).withTimeout(1),
            Commands.run(() -> {
                drive(0.0, 0.0, -1.0, false);
            }, this).withTimeout(1)
        );
    }

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

            double xVelocity = MathUtil.applyDeadband(xAxis.getAsDouble(), 0.01) * MAX_LINEAR_VELOCITY * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double yVelocity = MathUtil.applyDeadband(yAxis.getAsDouble(), 0.01) * MAX_LINEAR_VELOCITY * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double rotVelocity = MathUtil.applyDeadband(rotAxis.getAsDouble(), 0.01) * MAX_ANGULAR_VELOCITY * (slowMode ? ANGULAR_SLOW_MODE_MODIFIER : 1);
            drive(xVelocity, yVelocity, rotVelocity, FIELD_RELATIVE_DRIVE);

        }, this);
    }
    
}
