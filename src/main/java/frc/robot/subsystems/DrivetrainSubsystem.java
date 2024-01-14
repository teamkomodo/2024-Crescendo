package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
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

    private final Translation2d frontLeftPosition = new Translation2d(DRIVETRAIN_WIDTH / 2D, DRIVETRAIN_LENGTH / 2D); // All translations are relative to center of rotation
    private final Translation2d frontRightPosition = new Translation2d(DRIVETRAIN_WIDTH / 2D, -DRIVETRAIN_LENGTH / 2D);
    private final Translation2d backLeftPosition = new Translation2d(-DRIVETRAIN_WIDTH / 2D, DRIVETRAIN_LENGTH / 2D);
    private final Translation2d backRightPosition = new Translation2d(-DRIVETRAIN_WIDTH / 2D, -DRIVETRAIN_LENGTH / 2D);

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition);
    private final SwerveDriveOdometry odometry;
    private final Field2d field;

    private final HolonomicDriveController driveController = new HolonomicDriveController(
        new PIDController(1, 0, 0),
        new PIDController(1, 0, 0),
        new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCEL)));
    
    private final AHRS navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    private boolean slowMode = true; // Defaults to true -- robot should automatically be slow
    private double rotationOffsetRadians = 0.0;

    public DrivetrainSubsystem(Field2d field) {
        this.field = field;

        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        frontLeft = new NeoSwerveModule(
                FRONT_LEFT_DRIVE_MOTOR_ID,
                FRONT_LEFT_STEER_MOTOR_ID,
                FRONT_LEFT_STEER_ENCODER_ID,
                FRONT_LEFT_STEER_OFFSET,
                tab.getLayout("Front Left", BuiltInLayouts.kList).withSize(3, 7).withPosition(0, 0));
        
        frontRight = new NeoSwerveModule(
                FRONT_RIGHT_DRIVE_MOTOR_ID,
                FRONT_RIGHT_STEER_MOTOR_ID,
                FRONT_RIGHT_STEER_ENCODER_ID,
                FRONT_RIGHT_STEER_OFFSET,
                tab.getLayout("Front Right", BuiltInLayouts.kList).withSize(3, 7).withPosition(3, 0));

        backLeft = new NeoSwerveModule(
                BACK_LEFT_DRIVE_MOTOR_ID,
                BACK_LEFT_STEER_MOTOR_ID,
                BACK_LEFT_STEER_ENCODER_ID,
                BACK_LEFT_STEER_OFFSET,
                tab.getLayout("Back Left", BuiltInLayouts.kList).withSize(3, 7).withPosition(6, 0));
        
        backRight = new NeoSwerveModule(
                BACK_RIGHT_DRIVE_MOTOR_ID,
                BACK_RIGHT_STEER_MOTOR_ID,
                BACK_RIGHT_STEER_ENCODER_ID,
                BACK_RIGHT_STEER_OFFSET,
                tab.getLayout("Back Right", BuiltInLayouts.kList).withSize(3, 7).withPosition(9, 0));

        tab.add("Test Drivetrain", testDrivetrain()).withPosition(8, 0);

        tab.addNumber("Rotation", () -> (getAdjustedRotation().getDegrees()));

        odometry = new SwerveDriveOdometry(
                kinematics,
                navX.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
        resetPose(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)));
    }

    @Override
    public void periodic() {
        // does not need to use adjusted rotation, odometry handles it.
        odometry.update(navX.getRotation2d(), getSwervePositions());
        field.setRobotPose(getPose());
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
        return odometry.getPoseMeters();
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
        //does not need to be adjusted rotation, odometry handles this
        odometry.resetPosition(navX.getRotation2d(), getSwervePositions(), pose);
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

    public Command joystickDriveCommand(DoubleSupplier xAxis, DoubleSupplier yAxis, DoubleSupplier rotAxis) {
        return Commands.run(() -> {

            double xVelocity = MathUtil.applyDeadband(xAxis.getAsDouble(), 0.01) * MAX_LINEAR_VELOCITY * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double yVelocity = MathUtil.applyDeadband(yAxis.getAsDouble(), 0.01) * MAX_LINEAR_VELOCITY * (slowMode ? LINEAR_SLOW_MODE_MODIFIER : 1);
            double rotVelocity = MathUtil.applyDeadband(rotAxis.getAsDouble(), 0.01) * MAX_ANGULAR_VELOCITY * (slowMode ? ANGULAR_SLOW_MODE_MODIFIER : 1);
            drive(xVelocity, yVelocity, rotVelocity, FIELD_RELATIVE_DRIVE);

        }, this);
    }
    
}
