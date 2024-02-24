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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {

    private final CANSparkMax leftMotor;
    private final SparkPIDController leftMotorPidController;
    private final RelativeEncoder leftMotorEncoder;

    private final CANSparkMax rightMotor;
  
    private double p = 0.009;
    private double i = 0.000000001;
    private double d = 0.002;
    private double maxIAccum = 0;

    private boolean atMinPosition = false;
    private boolean atMaxPosition = false;

    private boolean leftMotorZeroed = false;
    private boolean rightMotorZeroed = false;

    private boolean atLeftSensor = false;
    private boolean atRightSensor = false;

    private final BooleanPublisher minPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("atminposition").publish();
    private final BooleanPublisher maxPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("atmaxposition").publish();
    private final DoublePublisher leftMotorPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("leftmotorposition").publish();

    public ClimberSubsystem() {
        leftMotor = new CANSparkMax(CLIMBER_MOTOR_1_ID, MotorType.kBrushless); // CHANGE DEVICE ID
        leftMotor.setInverted(false);
        leftMotor.setSmartCurrentLimit(80);
        
        leftMotorEncoder = leftMotor.getEncoder();
        leftMotorEncoder.setPosition(0);

        leftMotorPidController = leftMotor.getPIDController();
        leftMotorPidController.setP(p);
        leftMotorPidController.setI(i);
        leftMotorPidController.setD(d);
        leftMotorPidController.setIMaxAccum(maxIAccum, 0);
        leftMotorPidController.setReference(0, ControlType.kDutyCycle);

        // motor1BeamBreak = new DigitalInput(CLIMBER_MOTOR_1_BEAM_BREAK_ID);
        // motor2BeamBreak = new DigitalInput(CLIMBER_MOTOR_2_BEAM_BREAK_ID);
        
        rightMotor = new CANSparkMax(CLIMBER_MOTOR_2_ID, MotorType.kBrushless);
        rightMotor.follow(leftMotor, true);

      }
  
    @Override
    public void periodic() {
        checkMinPosition();
        checkMaxPosition();
        updateTelemetry();
    }
  
    public void teleopInit() {
        leftMotorEncoder.setPosition(0);
        leftMotorPidController.setReference(0, ControlType.kDutyCycle);
    } 

    // public void checkSensor() {
    //     atMotor1Limit = motor1BeamBreak.get();
    //     atMotor2Limit = motor2BeamBreak.get();

    //     if(atMotor1Limit) {
    //         motor1Encoder.setPosition(0);
    //         motor1Zeroed = true;
    //     }

    //     if(atMotor2Limit) {
    //         motor2Encoder.setPosition(0);
    //         motor2Zeroed = true;
    //     }

    //     if (atMotor1Limit && atMotor2Limit) {
    //         motor2.follow(motor1);
    //     }
    // }

    public void checkMinPosition() {
        if(!atMinPosition && leftMotorEncoder.getPosition() < CLIMBER_MIN_POSITION) {
            atMinPosition = true;
            setMotorPosition(CLIMBER_MIN_POSITION);
        } else if (leftMotorEncoder.getPosition() > CLIMBER_MIN_POSITION) {
            atMinPosition = false;
        }
    }

    public void checkMaxPosition() {
        if(!atMaxPosition && leftMotorEncoder.getPosition() > CLIMBER_MAX_POSITION) {
            atMaxPosition = true;
            setMotorPosition(CLIMBER_MAX_POSITION);
        } else if(leftMotorEncoder.getPosition() < CLIMBER_MAX_POSITION) {
            atMaxPosition = false;
        }
    }

    public Command climbPositionCommand() {
        return this.runOnce(() -> setMotorPosition(CLIMBER_PRE_CLIMB_POSITION));
    }

    public Command climb() {
        return this.runOnce(() -> setMotorPosition(CLIMBER_POST_CLIMB_POSITION));
    }

    public void updateTelemetry() {
        minPositionPublisher.set(atMinPosition);
        maxPositionPublisher.set(atMaxPosition);
        leftMotorPositionPublisher.set(leftMotorEncoder.getPosition());

    }

    public Command climberZeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setMotorDutyCycle(-0.3), this),
            Commands.waitUntil(() -> (leftMotorZeroed))
        );
    }
  
    public void setMotorDutyCycle(double dutyCycle) {
        if ((atMaxPosition && dutyCycle > 0) || (atMinPosition && dutyCycle < 0))
            return;
        leftMotor.set(dutyCycle);
    }

  
    public void setMotorVelocity(double velocity) {
        if ((atMaxPosition && velocity > 0) || (atMinPosition && velocity < 0))
            return;
        leftMotorPidController.setReference(velocity, ControlType.kVelocity);
    }

    public void setMotorPosition(Double position) {
        double clampedPosition = Math.max(Math.min(position, CLIMBER_MAX_POSITION), CLIMBER_MIN_POSITION);
        leftMotorPidController.setReference(clampedPosition, ControlType.kPosition);
    }
  
    public void holdMotorPosition() {
        setMotorPosition(leftMotorEncoder.getPosition());
    }
  
    public double getCurrent() {
        return (leftMotor.getOutputCurrent());
    }
  
}