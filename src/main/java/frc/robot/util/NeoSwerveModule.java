package frc.robot.util;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

import static frc.robot.Constants.*;

public class NeoSwerveModule implements SwerveModule{

    private final double driveP = 1.0;
    private final double driveI = 0;
    private final double driveD = 0;

    private final double steerP = 1.0;
    private final double steerI = 0;
    private final double steerD = 0;

    private final CANSparkMax driveMotor;
    private final CANSparkMax steerMotor;
    private final CANcoder steerAbsoluteEncoder;

    private final RelativeEncoder driveRelativeEncoder;
    private final RelativeEncoder steerRelativeEncoder;

    private final PIDController driveController = new PIDController(driveP, driveI, driveD);

    private final SparkPIDController steerController;
        
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 3.33); // NEO_KV * (60.0/1.0) * (1.0/DRIVE_REDUCTION) * (1/(WHEEL_DIAMETER * Math.PI))); // V/RPM = VM/motor rev => VM/motor rev * (seconds/minutes) * (motor revs/wheel revs) * (wheel revs/m) = V/m/s

    private SwerveModuleState desiredState;

    private double relativeSteerAdjustment = 0;
    // private double relativeSteerAdjustmentFactor = 0.1;

    public NeoSwerveModule(int driveMotorId, int steerMotorId, int steerAbsoluteEncoderId, double steerOffset, ShuffleboardContainer container) {
        this.driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        this.steerMotor = new CANSparkMax(steerMotorId, MotorType.kBrushless);
        this.steerAbsoluteEncoder = new CANcoder(steerAbsoluteEncoderId);
        this.desiredState = new SwerveModuleState(0.0, Rotation2d.fromRadians(0));

        driveRelativeEncoder = driveMotor.getEncoder();

        steerAbsoluteEncoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(Math.toDegrees(steerOffset)));

        steerRelativeEncoder = steerMotor.getEncoder();
        steerController = steerMotor.getPIDController();

        configureMotors();
        buildShuffleboard(container);
    }

    private void buildShuffleboard(ShuffleboardContainer container) {
        container.addNumber("Desired Velocity", () -> desiredState.speedMetersPerSecond);
        container.addNumber("Desired Rotation", () -> desiredState.angle.getDegrees());
        container.addNumber("Rotation Error", () -> Math.toDegrees(steerRelativeEncoder.getPosition()) - desiredState.angle.getDegrees());
        container.addNumber("Velocity Error", () -> driveRelativeEncoder.getVelocity() - desiredState.speedMetersPerSecond);

        container.addNumber("Raw Absolute Rotation", () -> steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
        // container.addNumber("Adjusted Absolute Rotation", () -> getAbsoluteModuleRotation().getDegrees());
        container.addNumber("Raw Relative Rotation", () -> Math.toDegrees(steerRelativeEncoder.getPosition()));
        // container.addNumber("Adjusted Relative Rotation", () -> getModuleRotation().getDegrees());
        container.addNumber("Velocity", () -> driveRelativeEncoder.getVelocity());
    }

    private void configureMotors() {
        driveMotor.setInverted(true);
        driveMotor.setIdleMode(IdleMode.kBrake);

        double wheelPositionConversionFactor = Math.PI * WHEEL_DIAMETER * DRIVE_REDUCTION; // motor rotations -> wheel travel in meters
        driveRelativeEncoder.setPositionConversionFactor(wheelPositionConversionFactor);
        driveRelativeEncoder.setVelocityConversionFactor(wheelPositionConversionFactor / 60); // motor RPM -> wheel speed in m/s

        steerMotor.setInverted(true);
        steerMotor.setIdleMode(IdleMode.kBrake);

        steerRelativeEncoder.setPositionConversionFactor(2 * Math.PI * STEER_REDUCTION); // motor rotations -> module rotation in radians
        steerRelativeEncoder.setVelocityConversionFactor(2 * Math.PI * STEER_REDUCTION / 60); // motor RPM -> module rad/s
        steerRelativeEncoder.setPosition(getAbsoluteModuleRotation().getRadians());

        steerController.setP(steerP);
        steerController.setI(steerI);
        steerController.setD(steerD);

        steerController.setPositionPIDWrappingEnabled(true);
        steerController.setPositionPIDWrappingMaxInput(Math.PI);
        steerController.setPositionPIDWrappingMinInput(-Math.PI);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveRelativeEncoder.getVelocity(), getModuleRotation());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveRelativeEncoder.getPosition(), getModuleRotation());
    }

    public void setDesiredState(SwerveModuleState desiredState) {

        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getModuleRotation());

        final double driveOutput = driveController.calculate(driveRelativeEncoder.getVelocity(), optimizedState.speedMetersPerSecond);
        final double driveFeedforward = this.driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        //System.out.println(driveFeedforward);
        driveMotor.setVoltage(driveOutput + driveFeedforward);
        steerController.setReference(optimizedState.angle.getRadians(), ControlType.kPosition);

        this.desiredState = optimizedState;
    }

    // private void correctRelativeEncoder() {
    //     double delta = getAbsoluteModuleRotation().getRadians()-getModuleRotation().getRadians();
    //     if(delta > Math.PI)
    //         delta -= 2 * Math.PI;

    //     if(delta < -180)
    //         delta += 2 * Math.PI;

    //     relativeSteerAdjustment += delta * relativeSteerAdjustmentFactor;

    // }

    public Rotation2d getModuleRotation() {
        return new Rotation2d(steerRelativeEncoder.getPosition() + relativeSteerAdjustment);
        // return new Rotation2d(MathUtil.angleModulus(steerRelativeEncoder.getPosition() + steerOffset + relativeSteerAdjustment)); // Handled by 
    }

    public Rotation2d getAbsoluteModuleRotation() {
        return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition().getValueAsDouble())));
        // return new Rotation2d(MathUtil.angleModulus(Math.toRadians(steerAbsoluteEncoder.getAbsolutePosition()) + steerOffset));
    }
    
}
