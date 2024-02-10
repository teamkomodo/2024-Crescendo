// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax motor1;
    private final SparkPIDController motor1PidController;
    private final RelativeEncoder motor1Encoder;

    // private final CANSparkMax motor2;
    // private final SparkPIDController motor2PidController;
    // private final RelativeEncoder motor2Encoder;
  
    private double p = 1.0;
    private double i = 0;
    private double d = 0;
    private double maxIAccum = 0;
  
    private double smoothCurrent = 0;
    private double filterConstant = 0.8;

    private Boolean atMinPosition = true;
    private Boolean atMaxPosition = false;

    private final BooleanPublisher minPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("minposition").publish();
    private final BooleanPublisher maxPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("maxposition").publish();
    private final DoublePublisher motor1DutyCyclePublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("motor1dutycycle").publish();
    //private final DoublePublisher motor2DutyCyclePublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("motor2dutycycle").publish();
    private final DoublePublisher motor1PositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("motor1position").publish();
    //private final DoublePublisher motor2PositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("motor2position").publish();

    public ClimberSubsystem() {
        motor1 = new CANSparkMax(CLIMBER_MOTOR_1_ID, MotorType.kBrushless); // CHANGE DEVICE ID
        motor1.restoreFactoryDefaults();
        motor1.setInverted(false);
        motor1.setSmartCurrentLimit(30);
        
        motor1Encoder = motor1.getEncoder();
        motor1Encoder.setPosition(0);

        motor1PidController = motor1.getPIDController();
        motor1PidController.setP(p);
        motor1PidController.setI(i);
        motor1PidController.setD(d);
        motor1PidController.setIMaxAccum(maxIAccum, 0);
        motor1PidController.setReference(0, ControlType.kDutyCycle);

        // motor2 = new CANSparkMax(CLIMBER_MOTOR_2_ID, MotorType.kBrushless); // CHANGE DEVICE ID
        // motor2.restoreFactoryDefaults();
        // motor2.setInverted(true);
        // motor2.setSmartCurrentLimit(30);
        
        // motor2Encoder = motor2.getEncoder();
        // motor2Encoder.setPosition(0);

        // motor2PidController = motor2.getPIDController();
        // motor2PidController.setP(p);
        // motor2PidController.setI(i);
        // motor2PidController.setD(d);
        // motor2PidController.setIMaxAccum(maxIAccum, 0);
        // motor2PidController.setReference(0, ControlType.kDutyCycle);
      }
  
    @Override
    public void periodic() {
        smoothCurrent = smoothCurrent * filterConstant + motor1.getOutputCurrent() * (1-filterConstant);
        // smoothCurrent = smoothCurrent * filterConstant + motor2.getOutputCurrent() * (1-filterConstant);
        checkMinPosition();
        checkMaxPosition();
        updateTelemetry();
    }
  
    public void teleopInit() {
        motor1PidController.setReference(0, ControlType.kDutyCycle);
        //motor2PidController.setReference(0, ControlType.kDutyCycle);
    } 

    public void checkMinPosition() {
        if(motor1Encoder.getPosition() < CLIMBER_MIN_POSITION) { //&& motor2Encoder.getPosition() < CLIMBER_MIN_POSITION) {
            atMinPosition = false;
        }
        if(atMinPosition) {
            setMotorDutyCycle(0);
        }
    }

    public void checkMaxPosition() {
        if(motor1Encoder.getPosition() > CLIMBER_MAX_POSITION) { //&& motor2Encoder.getPosition() > CLIMBER_MAX_POSITION) {
            atMaxPosition = false;
        }
        if(atMaxPosition) {
            setMotorDutyCycle(0);
        }
    }

    public Command climb() {
        return this.runOnce(() -> {
            motor1Encoder.setPosition(0);
            //motor2Encoder.setPosition(0);
            setMotorDutyCycle(0.3);
        });
    }

    public void updateTelemetry() {
        minPositionPublisher.set(atMinPosition);
        maxPositionPublisher.set(atMaxPosition);
        motor1DutyCyclePublisher.set(motor1.getOutputCurrent());
        //motor2DutyCyclePublisher.set(motor2.getOutputCurrent());
        motor1PositionPublisher.set(motor1Encoder.getPosition());
        //motor2PositionPublisher.set(motor2Encoder.getPosition());
    }
  
    public void setMotorDutyCycle(double dutyCycle) {
        motor1PidController.setReference(dutyCycle, ControlType.kDutyCycle);
        //motor2PidController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

  
    public void setMotorVelocity(double velocity) {
        motor1PidController.setReference(velocity, ControlType.kVelocity);
        //motor2PidController.setReference(velocity, ControlType.kVelocity);
    }
  
    public void holdMotorPosition() {
        motor1PidController.setReference(motor1Encoder.getPosition(), ControlType.kPosition);
        //motor2PidController.setReference(motor1Encoder.getPosition(), ControlType.kPosition);
    }
  
    public double getCurrent() {
        return (motor1.getOutputCurrent()); // + motor2.getOutputCurrent()) / 2;
    }
  
    public double getSmoothCurrent() {
        return smoothCurrent;
    }
}