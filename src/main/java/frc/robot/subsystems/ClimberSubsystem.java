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

    private final CANSparkMax motor1;
    private final SparkPIDController motor1PidController;
    private final RelativeEncoder motor1Encoder;

    // private final DigitalInput motor1BeamBreak;
    // private final DigitalInput motor2BeamBreak;

    private final CANSparkMax motor2;
    // private final SparkPIDController motor2PidController;
    // private final RelativeEncoder motor2Encoder;
  
    private double p = 0.009;
    private double i = 0.000000001;
    private double d = 0.002;
    private double maxIAccum = 0;
  
    private double smoothCurrent = 0;
    private double filterConstant = 0.8;

    private boolean atMinPosition = false;
    private boolean atMaxPosition = false;

    private boolean motor1Zeroed = false;
    private boolean motor2Zeroed = false;

    private boolean atMotor1Limit = false;
    private boolean atMotor2Limit = false;

    private final BooleanPublisher minPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("minposition").publish();
    private final BooleanPublisher maxPositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getBooleanTopic("maxposition").publish();
    private final DoublePublisher motor1PositionPublisher = NetworkTableInstance.getDefault().getTable("climber").getDoubleTopic("motor1position").publish();

    public ClimberSubsystem() {
        motor1 = new CANSparkMax(CLIMBER_MOTOR_1_ID, MotorType.kBrushless); // CHANGE DEVICE ID
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

        // motor1BeamBreak = new DigitalInput(CLIMBER_MOTOR_1_BEAM_BREAK_ID);
        // motor2BeamBreak = new DigitalInput(CLIMBER_MOTOR_2_BEAM_BREAK_ID);
        
        motor2 = new CANSparkMax(CLIMBER_MOTOR_2_ID, MotorType.kBrushless);
        motor2.setInverted(true);
        motor2.follow(motor1);

        // motor2Encoder = motor2.getEncoder();
        // motor2Encoder.setPosition(0);

        // motor2PidController = motor1.getPIDController();
        // motor2PidController.setP(p);
        // motor2PidController.setI(i);
        // motor2PidController.setD(d);
        // motor2PidController.setIMaxAccum(maxIAccum, 0);
        // motor2PidController.setReference(0, ControlType.kDutyCycle);
      }
  
    @Override
    public void periodic() {
        //checkMinPosition();
        //checkMaxPosition();
        updateTelemetry();
    }
  
    public void teleopInit() {
        motor1Encoder.setPosition(0);
        motor1PidController.setReference(0, ControlType.kDutyCycle);
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
        if(!atMinPosition && motor1Encoder.getPosition() < CLIMBER_MIN_POSITION) {
            atMinPosition = true;
            setMotorPosition(CLIMBER_MIN_POSITION);
        } else if (motor1Encoder.getPosition() > CLIMBER_MIN_POSITION) {
            atMinPosition = false;
        }
    }

    public void checkMaxPosition() {
        if(!atMaxPosition && motor1Encoder.getPosition() > CLIMBER_MAX_POSITION) {
            atMaxPosition = true;
            setMotorPosition(CLIMBER_MAX_POSITION);
        } else if(motor1Encoder.getPosition() < CLIMBER_MAX_POSITION) {
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
        motor1PositionPublisher.set(motor1Encoder.getPosition());

    }

    public Command climberZeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setMotorDutyCycle(-0.3), this),
            Commands.waitUntil(() -> (motor1Zeroed))
        );
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