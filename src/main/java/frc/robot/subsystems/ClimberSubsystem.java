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
import edu.wpi.first.networktables.NetworkTable;
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
    private final SparkPIDController rightMotorPidController;
    private final RelativeEncoder rightMotorEncoder;
  
    private double p = 0.0009;
    private double i = 0.000000001;
    private double d = 0.002;
    private double maxIAccum = 0;

    private boolean useSensors = true;
    private boolean useCodeStops = true;

    private boolean atMinPosition = false;
    private boolean atMaxPosition = false;

    private boolean leftMotorZeroed = false;
    private boolean rightMotorZeroed = false;
    private boolean climberZeroed = false;

    private boolean atLeftSensor = false;
    private boolean atRightSensor = false;

    private final DigitalInput leftSensor;
    private final DigitalInput rightSensor;

    private final NetworkTable climberTable = NetworkTableInstance.getDefault().getTable("climber");
    private final BooleanPublisher minPositionPublisher = climberTable.getBooleanTopic("atminposition").publish();
    private final BooleanPublisher maxPositionPublisher = climberTable.getBooleanTopic("atmaxposition").publish();
    private final BooleanPublisher leftSensorPublisher = climberTable.getBooleanTopic("leftsensor").publish();
    private final BooleanPublisher rightSensorPublisher = climberTable.getBooleanTopic("rightsensor").publish();
    private final BooleanPublisher leftMotorZeroedPublisher = climberTable.getBooleanTopic("leftmotorzeroed").publish();
    private final BooleanPublisher rightMotorZeroedPublisher = climberTable.getBooleanTopic("rightmotorzeroed").publish();
    private final DoublePublisher leftMotorPositionPublisher = climberTable.getDoubleTopic("leftmotorposition").publish();

    public ClimberSubsystem() {
        leftMotor = new CANSparkMax(CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless); // CHANGE DEVICE ID
        leftMotor.setInverted(true);
        leftMotor.setSmartCurrentLimit(80);
        
        leftMotorEncoder = leftMotor.getEncoder();
        leftMotorEncoder.setPosition(0);

        leftMotorPidController = leftMotor.getPIDController();
        leftMotorPidController.setP(p);
        leftMotorPidController.setI(i);
        leftMotorPidController.setD(d);
        leftMotorPidController.setIMaxAccum(maxIAccum, 0);
        leftMotorPidController.setReference(0, ControlType.kDutyCycle);

        leftSensor = new DigitalInput(CLIMBER_MOTOR_RIGHT_BEAM_BREAK_ID);
        rightSensor = new DigitalInput(CLIMBER_MOTOR_LEFT_BEAM_BREAK_ID);
        
        rightMotor = new CANSparkMax(CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless);
        rightMotor.setInverted(false);
        rightMotor.setSmartCurrentLimit(80);
        
        rightMotorEncoder = rightMotor.getEncoder();
        rightMotorEncoder.setPosition(0);

        rightMotorPidController = rightMotor.getPIDController();
        rightMotorPidController.setP(p);
        rightMotorPidController.setI(i);
        rightMotorPidController.setD(d);
        rightMotorPidController.setIMaxAccum(maxIAccum, 0);
        rightMotorPidController.setReference(0, ControlType.kDutyCycle);

      }
  
    @Override
    public void periodic() {
        checkSensors();
        checkMinPosition();
        checkMaxPosition();
        updateTelemetry();
    }
  
    public void teleopInit() {
        leftMotorEncoder.setPosition(0);
        leftMotorPidController.setReference(0, ControlType.kDutyCycle);
    } 

    public void checkSensors() {

        if(!useSensors) {
            leftMotorZeroed = true;
            rightMotorZeroed = true;
            return;
        }

        // left sensor rising edge
        if(!atLeftSensor && isLeftSensorTriggered()) {
            
            // if the motor is moving down, zero the encoder
            // always stop to motor, regardless of direction
            if(leftMotorEncoder.getVelocity() < 0) {
                leftMotorEncoder.setPosition(0);
                leftMotorZeroed = true;
                setMotorPosition(0);
            } else {
                setMotorPosition(leftMotorEncoder.getPosition());
            }
        }

        // left sensor falling edge
        if(atLeftSensor && !isLeftSensorTriggered()) {

            // if the motor is moving up set the encoder position
            if(leftMotorEncoder.getVelocity() > 0)
                leftMotorEncoder.setPosition(0);
        }

        // right sensor rising edge
        if(!atRightSensor && isRightSensorTriggered()) {
            rightMotorZeroed = true;
        }

        if (rightMotorZeroed && leftMotorZeroed) {
            climberZeroed = true;
            rightMotor.follow(leftMotor, true);
        }

        atLeftSensor = isLeftSensorTriggered();
        atRightSensor = isRightSensorTriggered();
    }

    public void checkMinPosition() {

        if(!useCodeStops) {
            atMinPosition = false;
            return;
        }

        if(!climberZeroed) {
            return;
        }

        if(!atMinPosition && leftMotorEncoder.getPosition() < CLIMBER_MIN_POSITION) {
            atMinPosition = true;
            setMotorPosition(CLIMBER_MIN_POSITION);
        } else if (leftMotorEncoder.getPosition() > CLIMBER_MIN_POSITION) {
            atMinPosition = false;
        }
    }

    public void checkMaxPosition() {

        if(!useCodeStops) {
            atMaxPosition = false;
            return;
        }

        if(!climberZeroed) {
            return;
        }

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
        leftSensorPublisher.set(isLeftSensorTriggered());
        rightSensorPublisher.set(isRightSensorTriggered());
        leftMotorZeroedPublisher.set(leftMotorZeroed);
        rightMotorZeroedPublisher.set(rightMotorZeroed);
    }

    public Command climberLeftZeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setLeftMotorDutyCycle(-0.2)),
            Commands.waitUntil(() -> (leftMotorZeroed)),
            Commands.runOnce(() -> setLeftMotorDutyCycle(0))
        );
    }

    public Command climberRightZeroCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> setRightMotorDutyCycle(-0.2)),
            Commands.waitUntil(() -> (rightMotorZeroed)),
            Commands.runOnce(() -> setRightMotorDutyCycle(0))
        );
    }

    public Command climberZeroCommand() {
        Command command = Commands.parallel(
            climberRightZeroCommand(),
            climberLeftZeroCommand()
        );

        command.addRequirements(this);
        return command;
    }
  
    public void setLeftMotorDutyCycle(double dutyCycle) {
        if ((atMaxPosition && dutyCycle > 0) || (atMinPosition && dutyCycle < 0 && leftMotorZeroed))
            return;
        leftMotor.set(dutyCycle);
    }

    public void setRightMotorDutyCycle(double dutyCycle) {
        if ((atMaxPosition && dutyCycle > 0) || (atMinPosition && dutyCycle < 0 && rightMotorZeroed))
            return;
        rightMotor.set(dutyCycle);
    }

  
    public void setMotorVelocity(double velocity) {
        if ((atMaxPosition && velocity > 0) || (atMinPosition && velocity < 0))
            return;
        leftMotorPidController.setReference(velocity, ControlType.kVelocity);
    }

    public void setMotorPosition(double position) {
        double clampedPosition = Math.max(Math.min(position, CLIMBER_MAX_POSITION), CLIMBER_MIN_POSITION);
        leftMotorPidController.setReference(clampedPosition, ControlType.kPosition);
    }
  
    public void holdMotorPosition() {
        setMotorPosition(leftMotorEncoder.getPosition());
    }
  
    public double getCurrent() {
        return (leftMotor.getOutputCurrent());
    }

    public boolean isLeftSensorTriggered() {
        // sensor sees tape when false
        return !leftSensor.get();
    }

    public boolean isRightSensorTriggered() {
        // sensor sees tape when false
        return rightSensor.get();
    }
  
}