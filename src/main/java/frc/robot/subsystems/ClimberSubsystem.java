// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import frc.robot.commands.OneSideProfiledClimbCommand;

import static frc.robot.Constants.*;

public class ClimberSubsystem extends SubsystemBase {

    private final boolean useSensors = false;
    private final boolean useCodeStops = true;

    private final CANSparkMax leftMotor;
    private final SparkPIDController leftMotorPidController;
    private final RelativeEncoder leftMotorEncoder;

    private final CANSparkMax rightMotor;
    private final SparkPIDController rightMotorPidController;
    private final RelativeEncoder rightMotorEncoder;
  
    private double p = 0.1;
    private double i = 5.0e-9;
    private double d = 0.002;
    private double maxIAccum = 0;

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
    private final BooleanPublisher leftMinPositionPublisher = climberTable.getBooleanTopic("left/atminposition").publish();
    private final BooleanPublisher leftMaxPositionPublisher = climberTable.getBooleanTopic("left/atmaxposition").publish();
    private final BooleanPublisher rightMinPositionPublisher = climberTable.getBooleanTopic("right/atminposition").publish();
    private final BooleanPublisher rightMaxPositionPublisher = climberTable.getBooleanTopic("right/atmaxposition").publish();
    private final BooleanPublisher leftSensorPublisher = climberTable.getBooleanTopic("left/sensor").publish();
    private final BooleanPublisher rightSensorPublisher = climberTable.getBooleanTopic("right/sensor").publish();
    private final BooleanPublisher leftMotorZeroedPublisher = climberTable.getBooleanTopic("left/motorzeroed").publish();
    private final BooleanPublisher rightMotorZeroedPublisher = climberTable.getBooleanTopic("right/motorzeroed").publish();
    private final BooleanPublisher climberZeroedPublisher = climberTable.getBooleanTopic("climberzeroed").publish();
    private final DoublePublisher leftMotorPositionPublisher = climberTable.getDoubleTopic("left/motorposition").publish();
    private final DoublePublisher rightMotorPositionPublisher = climberTable.getDoubleTopic("right/motorposition").publish();

    private final BooleanEntry useCodeStopsEntry = climberTable.getBooleanTopic("tuning/usecodestops").getEntry(useCodeStops);
    private final BooleanEntry useSensorsEntry = climberTable.getBooleanTopic("tuning/usesensors").getEntry(useSensors);

    private final DoubleEntry minPositionEntry = climberTable.getDoubleTopic("tuning/minposition").getEntry(CLIMBER_MIN_POSITION);
    private final DoubleEntry maxPositionEntry = climberTable.getDoubleTopic("tuning/maxposition").getEntry(CLIMBER_MAX_POSITION);
    private final DoubleEntry preClimbPositionEntry = climberTable.getDoubleTopic("tuning/preclimbposition").getEntry(CLIMBER_PRE_CLIMB_POSITION);
    private final DoubleEntry postClimbPositionEntry = climberTable.getDoubleTopic("tuning/postclimbposition").getEntry(CLIMBER_POST_CLIMB_POSITION);

    private final DoubleEntry extendVelocityEntry = climberTable.getDoubleTopic("tuning/extendvelocity").getEntry(CLIMBER_EXTEND_VELOCITY);
    private final DoubleEntry ascendVelocityEntry = climberTable.getDoubleTopic("tuning/ascendvelocity").getEntry(CLIMBER_ASCEND_VELOCITY);
    private final DoubleEntry hooksOffsetEntry = climberTable.getDoubleTopic("tuning/hooksoffset").getEntry(CLIMBER_HOOKS_OFFSET);

    private final NetworkButton leftClimberDownButton = new NetworkButton(climberTable.getBooleanTopic("left/downcommand"));
    private final NetworkButton leftClimberUpButton = new NetworkButton(climberTable.getBooleanTopic("left/upcommand"));
    private final NetworkButton rightClimberDownButton = new NetworkButton(climberTable.getBooleanTopic("right/downcommand"));
    private final NetworkButton rightClimberUpButton = new NetworkButton(climberTable.getBooleanTopic("right/upcommand"));

    public ClimberSubsystem() {
        leftMotor = new CANSparkMax(CLIMBER_MOTOR_LEFT_ID, MotorType.kBrushless); // CHANGE DEVICE ID
        leftMotor.setInverted(true);
        leftMotor.setSmartCurrentLimit(80);
        leftMotor.setSoftLimit(SoftLimitDirection.kForward, (float) CLIMBER_MAX_POSITION);
        leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

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
        rightMotor.setSoftLimit(SoftLimitDirection.kForward, (float) CLIMBER_MAX_POSITION);
        rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        
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

        minPositionEntry.set(CLIMBER_MIN_POSITION);
        maxPositionEntry.set(CLIMBER_MAX_POSITION);
        preClimbPositionEntry.set(CLIMBER_PRE_CLIMB_POSITION);
        postClimbPositionEntry.set(CLIMBER_POST_CLIMB_POSITION);

        extendVelocityEntry.set(CLIMBER_EXTEND_VELOCITY);
        ascendVelocityEntry.set(CLIMBER_ASCEND_VELOCITY);
        hooksOffsetEntry.set(CLIMBER_HOOKS_OFFSET);

        // buildHookAlignmentCommands();
    }

    // private void buildHookAlignmentCommands() {
    //     // These commands will show up in NT, they are meant to be used in the pit to align the climber hooks
    //     double speed = 10;
    //     leftClimberDownButton.whileTrue(Commands.sequence(
    //         Commands.runOnce(() -> setUseCodeStops(false)),
    //         new OneSideProfiledClimbCommand(this, -speed, true)
    //     ).finallyDo(() -> {
    //         leftMotor.set(0);
    //         leftMotorEncoder.setPosition(0);
    //         setUseCodeStops(true);
    //     }));
    //     climberTable.getBooleanTopic("left/downcommand").publish().set(false);
        
    //     leftClimberUpButton.whileTrue(Commands.sequence(
    //         Commands.runOnce(() -> setUseCodeStops(false)),
    //         new OneSideProfiledClimbCommand(this, speed, true)
    //     ).finallyDo(() -> {
    //         leftMotor.set(0);
    //         leftMotorEncoder.setPosition(0);
    //         setUseCodeStops(true);
    //     }));
    //     climberTable.getBooleanTopic("left/upcommand").publish().set(false);

    //     rightClimberDownButton.whileTrue(Commands.sequence(
    //         Commands.runOnce(() -> setUseCodeStops(false)),
    //         new OneSideProfiledClimbCommand(this, -speed, false)
    //     ).finallyDo(() -> {
    //         rightMotor.set(0);
    //         rightMotorEncoder.setPosition(0);
    //         setUseCodeStops(true);
    //     }));
    //     climberTable.getBooleanTopic("right/downcommand").publish().set(false);

    //     rightClimberUpButton.whileTrue(Commands.sequence(
    //         Commands.runOnce(() -> setUseCodeStops(false)),
    //         new OneSideProfiledClimbCommand(this, speed, false)
    //     ).finallyDo(() -> {
    //         rightMotor.set(0);
    //         rightMotorEncoder.setPosition(0);
    //         setUseCodeStops(true);
    //     }));
    //     climberTable.getBooleanTopic("right/upcommand").publish().set(false);

    // }

    public void teleopInit() {
        leftMotorEncoder.setPosition(0);
        leftMotorPidController.setReference(0, ControlType.kDutyCycle);
    }
    
    // Periodic methods

    @Override
    public void periodic() {
        // checkSensors();
        // checkMinPosition();
        // checkMaxPosition();
        updateTelemetry();
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
    } 

    // public void checkSensors() {

    //     if(!getUseSensors()) {
    //         leftMotorZeroed = true;
    //         rightMotorZeroed = true;
    //         climberZeroed = true;
    //         return;
    //     }

    //     // left sensor rising edge
    //     if(!atLeftSensor && isLeftSensorTriggered()) {
            
    //         // if the motor is moving down, zero the encoder
    //         // always stop to motor, regardless of direction
    //         if(leftMotorEncoder.getVelocity() < 0) {
    //             leftMotorEncoder.setPosition(0);
    //             leftMotorZeroed = true;
    //             setLeftMotorPosition(0);
    //         }
    //     }

    //     // left sensor falling edge
    //     if(atLeftSensor && !isLeftSensorTriggered()) {

    //         // if the motor is moving up set the encoder position
    //         if(leftMotorEncoder.getVelocity() > 0)
    //             leftMotorEncoder.setPosition(0);
    //             leftMotorZeroed = true;
    //     }

    //     // right sensor rising edge
    //     if(!atRightSensor && isRightSensorTriggered()) {
            
    //         // if the motor is moving down, zero the encoder
    //         // always stop to motor, regardless of direction
    //         if(rightMotorEncoder.getVelocity() < 0) {
    //             rightMotorEncoder.setPosition(0);
    //             rightMotorZeroed = true;
    //             setRightMotorPosition(0);
    //         }
    //     }

    //     // right sensor falling edge
    //     if(atRightSensor && !isRightSensorTriggered()) {

    //         // if the motor is moving up set the encoder position
    //         if(rightMotorEncoder.getVelocity() > 0)
    //             rightMotorEncoder.setPosition(0);
    //             rightMotorZeroed = true;
    //     }

    //     if (rightMotorZeroed && leftMotorZeroed && !climberZeroed) {
    //         climberZeroed = true;
    //     }
      
    //     atLeftSensor = isLeftSensorTriggered();
    //     atRightSensor = isRightSensorTriggered();
    // }

    // public void checkMinPosition() {

    //     if(!getUseCodeStops()) {
    //         atLeftMinPosition = false;
    //         return;
    //     }

    //     if(!climberZeroed) {
    //         return;
    //     }
        
    //     // Rising edge
    //     if(!atLeftMinPosition && leftMotorEncoder.getPosition() < getMinPosition()) {
    //         atLeftMinPosition = true;
    //         //setLeftMotorPosition(getMinPosition());
    //     } else if (leftMotorEncoder.getPosition() > getMinPosition()) {
    //         atLeftMinPosition = false;
    //     }

    //     if(!atRightMinPosition && rightMotorEncoder.getPosition() < getMinPosition()) {
    //         atRightMinPosition = true;
    //         //setRightMotorPosition(getMinPosition());
    //     } else if (rightMotorEncoder.getPosition() > getMinPosition()) {
    //         atRightMinPosition = false;
    //     }
    // }

    // public void checkMaxPosition() {

    //     if(!getUseCodeStops()) {
    //         atLeftMaxPosition = false;
    //         atRightMaxPosition = false;
    //         return;
    //     }

    //     if(!climberZeroed) {
    //         return;
    //     }

    //     if(!atLeftMaxPosition && leftMotorEncoder.getPosition() > getMaxPosition()) {
    //         atLeftMaxPosition = true;
    //         //setLeftMotorPosition(getMaxPosition());
    //     } else if(leftMotorEncoder.getPosition() < getMaxPosition()) {
    //         atLeftMaxPosition = false;
    //     }

    //     if(!atRightMaxPosition && rightMotorEncoder.getPosition() > getMaxPosition()) {
    //         atRightMaxPosition = true;
    //         //setRightMotorPosition(getMaxPosition());
    //     } else if(rightMotorEncoder.getPosition() < getMaxPosition()) {
    //         atRightMaxPosition = false;
    //     }
    // }

    // Commands

    public Command climbPositionCommand() {
        return this.runOnce(() -> setLeftMotorPosition(getPreClimbPosition()));
    }

    public Command climb() {
        return this.runOnce(() -> setLeftMotorPosition(getPostClimbPosition()));
    }

    public Command climbUpCommand() {
        return this.runEnd(() -> {
            setClimberVelocity(10);
        }, this::holdClimberPosition);
    }

    public Command climbDownCommand() {
        return this.runEnd(() -> {
            setClimberVelocity(-10);
        }, this::holdClimberPosition);
    }

    // public Command climberLeftZeroCommand() {
    //     return Commands.sequence(
    //         Commands.runOnce(() -> setLeftMotorDutyCycle(-0.2)),
    //         Commands.waitUntil(() -> (leftMotorZeroed)),
    //         Commands.runOnce(() -> setLeftMotorDutyCycle(0))
    //     );
    // }

    // public Command climberRightZeroCommand() {
    //     return Commands.sequence(
    //         Commands.runOnce(() -> setRightMotorDutyCycle(-0.2)),
    //         Commands.waitUntil(() -> (rightMotorZeroed)),
    //         Commands.runOnce(() -> setRightMotorDutyCycle(0))
    //     );
    // }

    // public Command climberZeroCommand() {
    //     Command command = Commands.parallel(
    //         climberRightZeroCommand(),
    //         climberLeftZeroCommand()
    //     );

    //     command.addRequirements(this);
    //     return command;
    // }

    // Control methods

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
        
        double clampedPosition = position;
        if(getUseCodeStops())
            clampedPosition = Math.max(Math.min(position, getMaxPosition()), getMinPosition());
        leftMotorPidController.setReference(clampedPosition, ControlType.kPosition);
    }

    public void setRightMotorPosition(double position) {
        if(!rightMotorZeroed)
            return;

        double clampedPosition = position;
        if(getUseCodeStops())
            clampedPosition = Math.max(Math.min(position, getMaxPosition()), getMinPosition());
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

    // Getters
  
    public double getCurrent() {
        return (leftMotor.getOutputCurrent());
    }

    public double getLeftMotorPosition() {
        return leftMotorEncoder.getPosition();
    }

    public double getRightMotorPosition() {
        return rightMotorEncoder.getPosition();
    }

    public boolean isLeftSensorTriggered() {
        // sensor sees tape when false
        return !leftSensor.get();
    }

    public boolean isRightSensorTriggered() {
        // sensor sees tape when false
        return rightSensor.get();
    }

    public boolean isClimberAtPosition(double position, double tolerance) {
        return position - rightMotorEncoder.getPosition() < tolerance && position - leftMotorEncoder.getPosition() < tolerance; 
    }

    public boolean isClimberStowed() {
        return atRightMinPosition && atLeftMinPosition;
    }

    public void setUseCodeStops(boolean useCodeStops) {
        useCodeStopsEntry.set(useCodeStops);
    }

    public void setUseSensors(boolean useSensors) {
        useSensorsEntry.set(useSensors);
    }

    // Get positions from NT, default to preset constants

    public boolean getUseCodeStops() {
        return useCodeStopsEntry.get(useCodeStops);
    }

    public boolean getUseSensors() {
        return useSensorsEntry.get(useSensors);
    }

    public double getMinPosition() {
        return !TUNING_MODE ? CLIMBER_MIN_POSITION : minPositionEntry.get(CLIMBER_MIN_POSITION);
    }

    public double getMaxPosition() {
        return !TUNING_MODE ? CLIMBER_MAX_POSITION : maxPositionEntry.get(CLIMBER_MAX_POSITION);
    }

    public double getPreClimbPosition() {
        return !TUNING_MODE ? CLIMBER_PRE_CLIMB_POSITION : preClimbPositionEntry.get(CLIMBER_PRE_CLIMB_POSITION);
    }

    public double getPostClimbPosition() {
        return !TUNING_MODE ? CLIMBER_POST_CLIMB_POSITION : postClimbPositionEntry.get(CLIMBER_POST_CLIMB_POSITION);
    }

    public double getExtendVelocity() {
        return !TUNING_MODE ? CLIMBER_EXTEND_VELOCITY : extendVelocityEntry.get(CLIMBER_EXTEND_VELOCITY);
    }

    public double getAscendVelocity() {
        return !TUNING_MODE ? CLIMBER_ASCEND_VELOCITY : ascendVelocityEntry.get(CLIMBER_ASCEND_VELOCITY);
    }

    public double getHooksOffset() {
        return !TUNING_MODE ? CLIMBER_HOOKS_OFFSET : hooksOffsetEntry.get(CLIMBER_HOOKS_OFFSET);
    }
  
}