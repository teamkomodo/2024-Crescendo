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

    private final CANSparkMax motor2;
  
    private double p = 0.009;
    private double i = 0.000000001;
    private double d = 0.002;
    private double maxIAccum = 0;
  
    private double smoothCurrent = 0;
    private double filterConstant = 0.8;

    private boolean atMinPosition = false;
    private boolean atMaxPosition = false;

    private final BooleanPublisher minPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("minposition").publish();
    private final BooleanPublisher maxPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("maxposition").publish();
    private final DoublePublisher motor1PositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("motor1position").publish();

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
        
        motor2 = new CANSparkMax(CLIMBER_MOTOR_2_ID, MotorType.kBrushless);
        motor2.restoreFactoryDefaults();
        motor2.setInverted(true);
        motor2.follow(motor1);
      }
  
    @Override
    public void periodic() {
        smoothCurrent = smoothCurrent * filterConstant + motor1.getOutputCurrent() * (1-filterConstant);
        checkMinPosition();
        checkMaxPosition();
        updateTelemetry();
    }
  
    public void teleopInit() {
        motor1Encoder.setPosition(0);
        motor1PidController.setReference(0, ControlType.kDutyCycle);
    } 

    public void checkMinPosition() {
        if(!atMinPosition && motor1Encoder.getPosition() < CLIMBER_MIN_POSITION) {
            atMinPosition = true;
            setMotorPosition(CLIMBER_MIN_POSITION);
        } else if (atMinPosition && motor1Encoder.getPosition() > CLIMBER_MIN_POSITION) {
            atMinPosition = false;
        }
    }

    public void checkMaxPosition() {
        if(!atMaxPosition && motor1Encoder.getPosition() > CLIMBER_MAX_POSITION) {
            atMaxPosition = true;
            setMotorPosition(CLIMBER_MAX_POSITION);
        } else if(atMaxPosition && motor1Encoder.getPosition() < CLIMBER_MAX_POSITION) {
            atMaxPosition = false;
        }
    }

    public Command climbPositionCommand() {
        return this.runOnce(() -> setMotorPosition(CLIMBER_PRE_CLIMB_POSITION));
    }

    public Command validPositionCommand() {
        return this.runOnce(() -> setMotorPosition(20.0));
    }

    public Command climb() {
        return this.runOnce(() -> setMotorPosition(CLIMBER_POST_CLIMB_POSITION));
    }

    public void updateTelemetry() {
        minPositionPublisher.set(atMinPosition);
        maxPositionPublisher.set(atMaxPosition);
        motor1PositionPublisher.set(motor1Encoder.getPosition());

    }
  
    public void setMotorDutyCycle(double dutyCycle) {
        if ((atMaxPosition && dutyCycle > 0) || (atMinPosition && dutyCycle < 0))
            return;
        motor1PidController.setReference(dutyCycle, ControlType.kDutyCycle);
    }

  
    public void setMotorVelocity(double velocity) {
        if ((atMaxPosition && velocity > 0) || (atMinPosition && velocity < 0))
            return;
        motor1PidController.setReference(velocity, ControlType.kVelocity);
    }

    public void setMotorPosition(Double position) {
        if (position < CLIMBER_MIN_POSITION || position > CLIMBER_MAX_POSITION)
            return;
        motor1PidController.setReference(position, ControlType.kPosition);
    }
  
    public void holdMotorPosition() {
        motor1PidController.setReference(motor1Encoder.getPosition(), ControlType.kPosition);
    }
  
    public double getCurrent() {
        return (motor1.getOutputCurrent());
    }
  
    public double getSmoothCurrent() {
        return smoothCurrent;
    }
}