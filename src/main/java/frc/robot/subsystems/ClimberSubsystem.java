// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.BooleanEntry;
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

    private boolean useSensors = false;
    private boolean useCodeStops = true;

    private boolean atLeftMinPosition = false;
    private boolean atLeftMaxPosition = false;

    private boolean atRightMinPosition = false;
    private boolean atRightMaxPosition = false;

    private boolean leftMotorZeroed = false;
    private boolean rightMotorZeroed = false;
    private boolean climberZeroed = false;

    private boolean atLeftSensor = false;
    private boolean atRightSensor = false;

    private final DigitalInput leftSensor;
    private final DigitalInput rightSensor;

    private final NetworkTable climberTable = NetworkTableInstance.getDefault().getTable("climber");
    private final BooleanPublisher leftMinPositionPublisher = climberTable.getBooleanTopic("atleftminposition").publish();
    private final BooleanPublisher leftMaxPositionPublisher = climberTable.getBooleanTopic("atleftmaxposition").publish();
    private final BooleanPublisher rightMinPositionPublisher = climberTable.getBooleanTopic("atrightminposition").publish();
    private final BooleanPublisher rightMaxPositionPublisher = climberTable.getBooleanTopic("atrightmaxposition").publish();
    private final BooleanPublisher leftSensorPublisher = climberTable.getBooleanTopic("leftsensor").publish();
    private final BooleanPublisher rightSensorPublisher = climberTable.getBooleanTopic("rightsensor").publish();
    private final BooleanPublisher leftMotorZeroedPublisher = climberTable.getBooleanTopic("leftmotorzeroed").publish();
    private final BooleanPublisher rightMotorZeroedPublisher = climberTable.getBooleanTopic("rightmotorzeroed").publish();
    private final BooleanPublisher climberZeroedPublisher = climberTable.getBooleanTopic("climberzeroed").publish();
    private final DoublePublisher leftMotorPositionPublisher = climberTable.getDoubleTopic("leftmotorposition").publish();
    private final DoublePublisher rightMotorPositionPublisher = climberTable.getDoubleTopic("rightmotorposition").publish();

    private final BooleanEntry useCodeStopsEntry = climberTable.getBooleanTopic("usecodestops").getEntry(useCodeStops);
    private final BooleanEntry useSensorsEntry = climberTable.getBooleanTopic("usesensors").getEntry(useSensors);


    public ClimberSubsystem() {
        leftMotor = new CANSparkMax(CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless); // CHANGE DEVICE ID
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
        
        rightMotor = new CANSparkMax(CLIMBER_MOTOR_RIGHT_ID, MotorType.kBrushless);
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

        useCodeStopsEntry.set(useCodeStops);
        useSensorsEntry.set(useSensors);
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
            climberZeroed = true;
            return;
        }

        // left sensor rising edge
        if(!atLeftSensor && isLeftSensorTriggered()) {
            
            // if the motor is moving down, zero the encoder
            // always stop to motor, regardless of direction
            if(leftMotorEncoder.getVelocity() < 0) {
                leftMotorEncoder.setPosition(0);
                leftMotorZeroed = true;
                setLeftMotorPosition(0);
            }
        }

        // left sensor falling edge
        if(atLeftSensor && !isLeftSensorTriggered()) {

            // if the motor is moving up set the encoder position
            if(leftMotorEncoder.getVelocity() > 0)
                leftMotorEncoder.setPosition(0);
                leftMotorZeroed = true;
        }

        // right sensor rising edge
        if(!atRightSensor && isRightSensorTriggered()) {
            
            // if the motor is moving down, zero the encoder
            // always stop to motor, regardless of direction
            if(rightMotorEncoder.getVelocity() < 0) {
                rightMotorEncoder.setPosition(0);
                rightMotorZeroed = true;
                setRightMotorPosition(0);
            }
        }

        // right sensor falling edge
        if(atRightSensor && !isRightSensorTriggered()) {

            // if the motor is moving up set the encoder position
            if(rightMotorEncoder.getVelocity() > 0)
                rightMotorEncoder.setPosition(0);
                rightMotorZeroed = true;
        }

        if (rightMotorZeroed && leftMotorZeroed && !climberZeroed) {
            climberZeroed = true;
        }
      
        atLeftSensor = isLeftSensorTriggered();
        atRightSensor = isRightSensorTriggered();
    }

    public void checkMinPosition() {

        if(!useCodeStops) {
            atLeftMinPosition = false;
            return;
        }

        if(!climberZeroed) {
            return;
        }
        
        // Rising edge
        if(!atLeftMinPosition && leftMotorEncoder.getPosition() < CLIMBER_MIN_POSITION) {
            atLeftMinPosition = true;
            setLeftMotorPosition(CLIMBER_MIN_POSITION);
        } else if (leftMotorEncoder.getPosition() > CLIMBER_MIN_POSITION) {
            atLeftMinPosition = false;
        }

        if(!atRightMinPosition && rightMotorEncoder.getPosition() < CLIMBER_MIN_POSITION) {
            atRightMinPosition = true;
            setRightMotorPosition(CLIMBER_MIN_POSITION);
        } else if (rightMotorEncoder.getPosition() > CLIMBER_MIN_POSITION) {
            atRightMinPosition = false;
        }
    }

    public void checkMaxPosition() {

        if(!useCodeStops) {
            atLeftMaxPosition = false;
            atRightMaxPosition = false;
            return;
        }

        if(!climberZeroed) {
            return;
        }

        if(!atLeftMaxPosition && leftMotorEncoder.getPosition() > CLIMBER_MAX_POSITION) {
            atLeftMaxPosition = true;
            setLeftMotorPosition(CLIMBER_MAX_POSITION);
        } else if(leftMotorEncoder.getPosition() < CLIMBER_MAX_POSITION) {
            atLeftMaxPosition = false;
        }

        if(!atRightMaxPosition && rightMotorEncoder.getPosition() > CLIMBER_MAX_POSITION) {
            atRightMaxPosition = true;
            setRightMotorPosition(CLIMBER_MAX_POSITION);
        } else if(rightMotorEncoder.getPosition() < CLIMBER_MAX_POSITION) {
            atRightMaxPosition = false;
        }
    }

    public Command climbPositionCommand() {
        return this.runOnce(() -> setLeftMotorPosition(CLIMBER_PRE_CLIMB_POSITION));
    }

    public Command climb() {
        return this.runOnce(() -> setLeftMotorPosition(CLIMBER_POST_CLIMB_POSITION));
    }

    public void updateTelemetry() {
        leftMinPositionPublisher.set(atLeftMinPosition);
        leftMaxPositionPublisher.set(atLeftMaxPosition);
        rightMinPositionPublisher.set(atRightMinPosition);
        rightMaxPositionPublisher.set(atRightMaxPosition);
        leftSensorPublisher.set(isLeftSensorTriggered());
        rightSensorPublisher.set(isRightSensorTriggered());
        leftMotorZeroedPublisher.set(leftMotorZeroed);
        rightMotorZeroedPublisher.set(rightMotorZeroed);
        climberZeroedPublisher.set(climberZeroed);
        leftMotorPositionPublisher.set(leftMotorEncoder.getPosition());
        rightMotorPositionPublisher.set(rightMotorEncoder.getPosition());

        boolean newUseCodeStops = useCodeStopsEntry.get(useCodeStops);
        if(useCodeStops != newUseCodeStops) {
            setUseCodeStops(newUseCodeStops);
        }

        boolean newUseSensors = useSensorsEntry.get(useSensors);
        if(useCodeStops != newUseSensors) {
            setUseCodeStops(newUseSensors);
        }
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

    public void setClimberDutyCycle(double dutyCycle) {
        if (!climberZeroed)
            return;
        setLeftMotorDutyCycle(dutyCycle);
        setRightMotorDutyCycle(dutyCycle);
    }
  
    public void setLeftMotorDutyCycle(double dutyCycle) {
        if ((atLeftMaxPosition && dutyCycle > 0) || (atLeftMinPosition && dutyCycle < 0 && leftMotorZeroed))
            return;
        leftMotor.set(dutyCycle);
    }

    public void setRightMotorDutyCycle(double dutyCycle) {
        if ((atRightMaxPosition && dutyCycle > 0) || (atRightMinPosition && dutyCycle < 0 && rightMotorZeroed))
            return;
        rightMotor.set(dutyCycle);
    }

    public void setClimberVelocity(double velocity) {
        if (!climberZeroed)
            return;
        setLeftMotorVelocity(velocity);
        setRightMotorVelocity(velocity);
    }
  
    public void setLeftMotorVelocity(double velocity) {
        if ((atLeftMaxPosition && velocity > 0) || (atLeftMinPosition && velocity < 0) || !leftMotorZeroed)
            return;
        leftMotorPidController.setReference(velocity, ControlType.kVelocity);
    }

    public void setRightMotorVelocity(double velocity) {
        if ((atRightMaxPosition && velocity > 0) || (atRightMinPosition && velocity < 0) || !rightMotorZeroed)
            return;
        rightMotorPidController.setReference(velocity, ControlType.kVelocity);
    }

    public void setClimberPosition(double position) {
        if (!climberZeroed)
            return;
        setLeftMotorPosition(position);
        setRightMotorPosition(position);
    }

    public void setLeftMotorPosition(double position) {
        if(!leftMotorZeroed)
            return;
        double clampedPosition = Math.max(Math.min(position, CLIMBER_MAX_POSITION), CLIMBER_MIN_POSITION);
        leftMotorPidController.setReference(clampedPosition, ControlType.kPosition);
    }

    public void setRightMotorPosition(double position) {
        if(!rightMotorZeroed)
            return;
        double clampedPosition = Math.max(Math.min(position, CLIMBER_MAX_POSITION), CLIMBER_MIN_POSITION);
        rightMotorPidController.setReference(clampedPosition, ControlType.kPosition);
    }

    public void holdClimberPosition() {
        if (!climberZeroed)
            return;
        holdLeftMotorPosition();
        holdRightMotorPosition();
    }
  
    public void holdLeftMotorPosition() {
        setLeftMotorPosition(leftMotorEncoder.getPosition());
    }

    public void holdRightMotorPosition() {
        setRightMotorPosition(rightMotorEncoder.getPosition());
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

    public void setUseSensors(boolean useSensors) {
        this.useSensors = useSensors;
    }

    public void setUseCodeStops(boolean useCodeStops) {
        this.useCodeStops = useCodeStops;
    }
  
}