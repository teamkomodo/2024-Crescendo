// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
    
    // 9:1 reduction
    // 5.53 in circumference
    
    private static final double ELEVATOR_INCHES_PER_REVOLUTION = 0; //3.53D/9.0D
    
    // Network Tables
    private final NetworkTable armTable = NetworkTableInstance.getDefault().getTable("arm");
    
    private final StringPublisher currentCommandPublisher = armTable.getStringTopic("currentcommand").publish();
    
    private final BooleanPublisher elevatorZeroedPublisher = armTable.getBooleanTopic("elevator/zeroed").publish();
    private final DoublePublisher elevatorPositionPublisher = armTable.getDoubleTopic("elevator/position").publish();
    private final BooleanPublisher elevatorSwitchPublisher = armTable.getBooleanTopic("elevator/switch").publish();
    private final BooleanPublisher atElevatorMinPublisher = armTable.getBooleanTopic("elevator/atmin").publish();
    private final BooleanPublisher atElevatorMaxPublisher = armTable.getBooleanTopic("elevator/atmax").publish();
    
    private final BooleanPublisher jointZeroedPublisher = armTable.getBooleanTopic("joint/zeroed").publish();
    private final DoublePublisher jointPositionPublisher = armTable.getDoubleTopic("joint/position").publish();
    private final BooleanPublisher jointMiddleSwitchPublisher = armTable.getBooleanTopic("joint/middleswitch").publish();
    private final BooleanPublisher jointBottomSwitchPublisher = armTable.getBooleanTopic("joint/bottomswitch").publish();
    private final BooleanPublisher atJointMinPublisher = armTable.getBooleanTopic("joint/atmin").publish();
    private final BooleanPublisher atJointMaxPublisher = armTable.getBooleanTopic("joint/atmax").publish();
    
    private final DoublePublisher jointCommandedPositionPublisher = armTable.getDoubleTopic("joint/commandedposition").publish();
    private final DoublePublisher elevatorCommandedPositionPublisher = armTable.getDoubleTopic("elevator/commandedposition").publish();
    
    // NT entries for tuning
    private final DoubleEntry jointMinPositionEntry = armTable.getDoubleTopic("tuning/joint/minposition").getEntry(JOINT_MIN_POSITION);
    private final DoubleEntry jointMaxPositionEntry = armTable.getDoubleTopic("tuning/joint/maxposition").getEntry(JOINT_MAX_POSITION);
    private final DoubleEntry jointStowPositionEntry = armTable.getDoubleTopic("tuning/joint/stowposition").getEntry(JOINT_STOW_POSITION);
    private final DoubleEntry jointAmpPositionEntry = armTable.getDoubleTopic("tuning/joint/ampposition").getEntry(JOINT_AMP_POSITION);
    private final DoubleEntry jointSpeakerPositionEntry = armTable.getDoubleTopic("tuning/joint/speakerposition").getEntry(JOINT_SPEAKER_POSITION);
    private final DoubleEntry jointTrapPositionEntry = armTable.getDoubleTopic("tuning/joint/trapposition").getEntry(JOINT_TRAP_POSITION);
    private final DoubleEntry jointIntakePositionEntry = armTable.getDoubleTopic("tuning/joint/intakeposition").getEntry(JOINT_INTAKE_POSITION);
    private final DoubleEntry jointPreIntakePositionEntry = armTable.getDoubleTopic("tuning/joint/preintakeposition").getEntry(JOINT_PRE_INTAKE_POSITION);
    
    private final DoubleEntry elevatorMinPositionEntry = armTable.getDoubleTopic("tuning/elevator/minposition").getEntry(ELEVATOR_MIN_POSITION);
    private final DoubleEntry elevatorMaxPositionEntry = armTable.getDoubleTopic("tuning/elevator/maxposition").getEntry(ELEVATOR_MAX_POSITION);
    private final DoubleEntry elevatorStowPositionEntry = armTable.getDoubleTopic("tuning/elevator/stowposition").getEntry(ELEVATOR_STOW_POSITION);
    private final DoubleEntry elevatorAmpPositionEntry = armTable.getDoubleTopic("tuning/elevator/ampposition").getEntry(ELEVATOR_AMP_POSITION);
    private final DoubleEntry elevatorSpeakerPositionEntry = armTable.getDoubleTopic("tuning/elevator/speakerposition").getEntry(ELEVATOR_SPEAKER_POSITION);
    private final DoubleEntry elevatorTrapPositionEntry = armTable.getDoubleTopic("tuning/elevator/trapposition").getEntry(ELEVATOR_TRAP_POSITION);
    private final DoubleEntry elevatorIntakePositionEntry = armTable.getDoubleTopic("tuning/elevator/intakeposition").getEntry(ELEVATOR_INTAKE_POSITION);
    
    private final DoubleEntry elevatorPEntry, elevatorIEntry, elevatorDEntry;
    private final DoubleEntry jointPEntry, jointIEntry, jointDEntry;
    
    private final CANSparkMax elevatorMotor;
    private final DigitalInput elevatorZeroLimitSwitch;
    private final SparkPIDController elevatorPidController;
    private final RelativeEncoder elevatorEncoder;
    
    private double elevatorP = 1.0;
    private double elevatorI = 1.0e-6;
    private double elevatorD = 0.7;
    
    private double elevatorSmartMotionMaxVel = 2000;
    private double elevatorSmartMotionMaxAccel = 1000;
    private double elevatorSmartMotionMinVel = 0;
    private double elevatorSmartMotionAllowedClosedLoopError = 1;
    
    private int elevatorHoldingCurrentLimit = 30;
    private int elevatorRunningCurrentLimit = 60;
    
    private double jointCommandedPosition = 0;
    private double elevatorCommandedPosition = 0;
    
    private boolean atElevatorLimitSwitch = false;
    private boolean atElevatorLimitSwitchAtLastCheck = false;
    private boolean elevatorZeroed = false;
    
    private boolean atElevatorMaxLimit = false;
    private boolean atElevatorMinLimit = false;
    
    //var
    private final CANSparkMax jointMotor;
    private final SparkPIDController jointPidController;
    private final RelativeEncoder jointEncoder;
    private final DigitalInput jointMiddleReverseSwitch;
    private final DigitalInput jointBottomReverseSwitch;
    
    private final CANSparkMax jointSecondMotor;
    
    private double jointP = 0.05;
    private double jointI = 0.000000005;
    private double jointD = 3;
    private double jointMaxIAccum = 0;
    
    private boolean jointZeroed = false;
    
    private boolean atJointMiddleLimitSwitch = false;
    private boolean atJointMiddleLimitSwitchAtLastCheck = true;
    
    private boolean atJointBottomLimitSwitch = false;
    private boolean atJointBottomLimitSwitchAtLastCheck = true;
    
    private boolean atJointMaxLimit = false;
    private boolean atJointMinLimit = false;
    
    private boolean useJointCodeStops = true;
    private boolean useJointSensors = true;
    private boolean useElevatorCodeStops = true;
    private boolean useElevatorSensors = true;
    
    public ArmSubsystem() {
        jointMotor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
        jointMotor.setInverted(false);
        jointMotor.setSmartCurrentLimit(50);
        
        jointMiddleReverseSwitch = new DigitalInput(JOINT_MIDDLE_ZERO_SWITCH_CHANNEL);
        jointBottomReverseSwitch = new DigitalInput(JOINT_BOTTOM_ZERO_SWITCH_CHANNEL);
        
        jointEncoder = jointMotor.getEncoder();
        jointEncoder.setPosition(JOINT_STARTING_POSITION);
        
        jointPidController = jointMotor.getPIDController();
        jointPidController.setP(jointP);
        jointPidController.setI(jointI);
        jointPidController.setD(jointD);
        jointPidController.setIMaxAccum(jointMaxIAccum, 0);
        jointPidController.setReference(0, ControlType.kPosition);
        
        jointSecondMotor = new CANSparkMax(JOINT_SECOND_MOTOR_ID, MotorType.kBrushless);
        jointSecondMotor.setInverted(true);
        jointSecondMotor.follow(jointMotor);
        
        jointPEntry = armTable.getDoubleTopic("tuning/joint/kP").getEntry(jointP);
        jointIEntry = armTable.getDoubleTopic("tuning/joint/kI").getEntry(jointI);
        jointDEntry = armTable.getDoubleTopic("tuning/joint/kD").getEntry(jointD);
        
        
        elevatorZeroLimitSwitch = new DigitalInput(ELEVATOR_ZERO_SWITCH_CHANNEL);
        
        elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
        elevatorMotor.setInverted(false);
        elevatorMotor.setSmartCurrentLimit(elevatorHoldingCurrentLimit, elevatorRunningCurrentLimit);
        
        elevatorEncoder = elevatorMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(ELEVATOR_INCHES_PER_REVOLUTION);
        
        elevatorPidController = elevatorMotor.getPIDController();
        elevatorPidController.setP(elevatorP);
        elevatorPidController.setI(elevatorI);
        elevatorPidController.setD(elevatorD);
        elevatorPidController.setSmartMotionMaxVelocity(elevatorSmartMotionMaxVel,0);
        elevatorPidController.setSmartMotionMaxAccel(elevatorSmartMotionMaxAccel, 0);
        elevatorPidController.setSmartMotionMinOutputVelocity(elevatorSmartMotionMinVel,0);
        elevatorPidController.setSmartMotionAllowedClosedLoopError(elevatorSmartMotionAllowedClosedLoopError, 0);
        elevatorPidController.setOutputRange(-1.0, 1.0);
        
        elevatorPEntry = armTable.getDoubleTopic("tuning/elevator/kP").getEntry(elevatorP);
        elevatorIEntry = armTable.getDoubleTopic("tuning/elevator/kI").getEntry(elevatorI);
        elevatorDEntry = armTable.getDoubleTopic("tuning/elevator/kD").getEntry(elevatorD);

        // Publish values so the entries appear in NT

        jointMinPositionEntry.set(JOINT_MIN_POSITION);
        jointMaxPositionEntry.set(JOINT_MAX_POSITION);
        jointStowPositionEntry.set(JOINT_STOW_POSITION);
        jointAmpPositionEntry.set(JOINT_AMP_POSITION);
        jointSpeakerPositionEntry.set(JOINT_SPEAKER_POSITION);
        jointTrapPositionEntry.set(JOINT_TRAP_POSITION);
        jointIntakePositionEntry.set(JOINT_INTAKE_POSITION);
        jointPreIntakePositionEntry.set(JOINT_PRE_INTAKE_POSITION);

        elevatorMinPositionEntry.set(ELEVATOR_MIN_POSITION);
        elevatorMaxPositionEntry.set(ELEVATOR_MAX_POSITION);
        elevatorStowPositionEntry.set(ELEVATOR_STOW_POSITION);
        elevatorAmpPositionEntry.set(ELEVATOR_AMP_POSITION);
        elevatorSpeakerPositionEntry.set(ELEVATOR_SPEAKER_POSITION);
        elevatorTrapPositionEntry.set(ELEVATOR_TRAP_POSITION);
        elevatorIntakePositionEntry.set(ELEVATOR_INTAKE_POSITION);
        
        jointPEntry.set(jointP);
        jointIEntry.set(jointI);
        jointDEntry.set(jointD);

        elevatorPEntry.set(elevatorP);
        elevatorIEntry.set(elevatorI);
        elevatorDEntry.set(elevatorD);

    }
    
    public void teleopInit() {
        elevatorMotor.set(0);
        jointMotor.set(0);
    }
    
    // Periodic methods
    
    @Override
    public void periodic() {
        checkJointSensors();
        checkElevatorSensors();
        checkJointCodeStops();
        checkElevatorCodeStops();
        updateTelemetry();
        updateControlConstants();
    }
    
    private void updateTelemetry() {
        jointZeroedPublisher.set(jointZeroed);
        jointPositionPublisher.set(jointEncoder.getPosition());
        jointMiddleSwitchPublisher.set(isJointMiddleLimitSwitchTriggered());
        jointBottomSwitchPublisher.set(isJointBottomLimitSwitchTriggered());
        atJointMinPublisher.set(atJointMinLimit);
        atJointMaxPublisher.set(atJointMaxLimit);
        
        elevatorZeroedPublisher.set(elevatorZeroed);
        elevatorPositionPublisher.set(elevatorEncoder.getPosition());
        elevatorSwitchPublisher.set(isElevatorLimitSwitchTriggered());
        atElevatorMinPublisher.set(atElevatorMinLimit);
        atElevatorMaxPublisher.set(atElevatorMaxLimit);
        
        jointCommandedPositionPublisher.set(jointCommandedPosition);
        elevatorCommandedPositionPublisher.set(elevatorCommandedPosition);
        
        currentCommandPublisher.set(getCurrentCommand() != null? getCurrentCommand().getName() : "null");
    }
    
    private void updateControlConstants() {
        double newElevatorP = elevatorPEntry.get(elevatorP);
        if(newElevatorP != elevatorP) {
            elevatorP = newElevatorP;
            elevatorPidController.setP(elevatorP);
        }
        
        double newElevatorI = elevatorIEntry.get(elevatorI);
        if(newElevatorI != elevatorI) {
            elevatorI = newElevatorI;
            elevatorPidController.setI(elevatorI);
        }
        
        double newElevatorD = elevatorDEntry.get(elevatorD);
        if(newElevatorD != elevatorD) {
            elevatorD = newElevatorD;
            elevatorPidController.setD(elevatorD);
        }
        
        double newJointP = jointPEntry.get(jointP);
        if(newJointP != jointP) {
            jointP = newJointP;
            jointPidController.setP(jointP);
        }
        
        double newJointI = jointIEntry.get(jointI);
        if(newJointI != jointI) {
            jointI = newJointI;
            jointPidController.setI(jointI);
        }
        
        double newJointD = jointDEntry.get(jointD);
        if(newJointD != jointD) {
            jointD = newJointD;
            jointPidController.setD(jointD);
        }
        
    }
    
    private void checkElevatorSensors() {
        
        if(!useElevatorSensors)
        return;
        
        atElevatorLimitSwitchAtLastCheck = atElevatorLimitSwitch;
        atElevatorLimitSwitch = isElevatorLimitSwitchTriggered();
        
        if(atElevatorLimitSwitch) {
            elevatorEncoder.setPosition(0);
            elevatorZeroed = true;
        }
        
        if(!atElevatorLimitSwitchAtLastCheck && atElevatorLimitSwitch) { // Rising edge
            elevatorEncoder.setPosition(0);
        } else if(atElevatorLimitSwitchAtLastCheck && !atElevatorLimitSwitch) { // Falling edge
            elevatorEncoder.setPosition(0);
        }
    }
    
    private void checkJointSensors() {
        
        if(!useJointSensors)
        return;
        
        double jointVelocity = jointEncoder.getVelocity();
        
        atJointBottomLimitSwitchAtLastCheck = atJointBottomLimitSwitch;
        atJointMiddleLimitSwitchAtLastCheck = atJointMiddleLimitSwitch;
        atJointBottomLimitSwitch = isJointBottomLimitSwitchTriggered();
        atJointMiddleLimitSwitch = isJointMiddleLimitSwitchTriggered();
        
        if (atJointMiddleLimitSwitch && !jointZeroed) {
            jointEncoder.setPosition(JOINT_MIDDLE_SWITCH_TOP_POSITION);
            jointZeroed = true;
        }
        
        // Falling edge of middle switch
        if(atJointMiddleLimitSwitchAtLastCheck && !atJointMiddleLimitSwitch) {
            // Update encoder reading with known position
            jointEncoder.setPosition((jointVelocity > 0? JOINT_MIDDLE_SWITCH_TOP_POSITION : JOINT_MIDDLE_SWITCH_BOTTOM_POSITION));
        }
        
        // Rising edge of middle switch
        if(!atJointMiddleLimitSwitchAtLastCheck && atJointMiddleLimitSwitch) {
            // Update encoder reading with known position
            jointEncoder.setPosition((jointVelocity < 0? JOINT_MIDDLE_SWITCH_TOP_POSITION : JOINT_MIDDLE_SWITCH_BOTTOM_POSITION));
        }
        
        if (atJointBottomLimitSwitch && !jointZeroed) {
            jointEncoder.setPosition(JOINT_BOTTOM_SWITCH_POSITION);
            jointZeroed = true;
        }
        
        // Falling edge of bottom switch
        if(atJointBottomLimitSwitchAtLastCheck && !atJointBottomLimitSwitch) {
            // Update encoder reading with known position
            jointEncoder.setPosition(JOINT_BOTTOM_SWITCH_POSITION);
        }
        
        // Rising edge of bottom switch
        if(!atJointBottomLimitSwitchAtLastCheck && atJointBottomLimitSwitch) {
            // Update encoder reading with known position
            jointEncoder.setPosition(JOINT_BOTTOM_SWITCH_POSITION);
        }
    }
    
    private void checkJointCodeStops() {
        
        if(!useJointCodeStops)
        return;
        
        // Min Pos Rising edge
        if(!atJointMinLimit && jointEncoder.getPosition() < getJointMinPosition() && jointZeroed) { //Rising edge
            atJointMinLimit = true;
            setJointPosition(getJointMinPosition());
        } else if(jointEncoder.getPosition() > getJointMinPosition()) {
            atJointMinLimit = false;
        }
        
        // Max Pos Rising edge
        if(!atJointMaxLimit && jointEncoder.getPosition() > getJointMaxPosition() && jointZeroed) {
            atJointMaxLimit = true;
            setJointPosition(getJointMaxPosition());
        } else if(jointEncoder.getPosition() < getJointMaxPosition()) {
            atJointMaxLimit = false;
        }
        
    }
    
    private void checkElevatorCodeStops() {
        
        if(!useElevatorCodeStops)
        return;
        
        // Min Pos Rising edge
        if(!atElevatorMinLimit && elevatorEncoder.getPosition() < getElevatorMinPosition() && elevatorZeroed) { //Rising edge
            atElevatorMinLimit = true;
            setElevatorPosition(getElevatorMinPosition());
        } else if(elevatorEncoder.getPosition() > getElevatorMinPosition()) {
            atElevatorMinLimit = false;
        }
        
        // Max Pos Rising edge
        if(!atElevatorMaxLimit && elevatorEncoder.getPosition() > getElevatorMaxPosition() && elevatorZeroed) { //Rising edge
            atElevatorMaxLimit = true;
            setElevatorPosition(getElevatorMaxPosition());
        } else if(elevatorEncoder.getPosition() < getElevatorMaxPosition()) {
            atElevatorMaxLimit = false;
        }
    }
    
    // Control methods

    public void setJointMotorPercent(double percent) {
        //if the speed is negative, we cannot move if we're at the min limit AND the joint is zeroed
        if(percent < 0 && atJointMinLimit && jointZeroed)
        return;
        
        // if the speed is positive, we cannot move if we're at the max limit OR the joint is not zeroed
        if(percent > 0 && (atJointMaxLimit || !jointZeroed))
        return;
        
        jointCommandedPosition = -1;
        jointPidController.setReference(percent * 0.5, ControlType.kDutyCycle);
    }
    
    public void setElevatorMotorPercent(double percent) {
        //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
        if(atElevatorMinLimit && percent < 0 && elevatorZeroed)
        return;
        
        //at max or not yet zeroed and attempting to increase
        if((atElevatorMaxLimit || !elevatorZeroed) && percent > 0)
        return;
        
        elevatorCommandedPosition = -1;
        elevatorPidController.setReference(percent * 0.5, ControlType.kDutyCycle);
    }
    
    public void setJointPosition(double position) {
        
        // Not zeroed and moving away from limit switch
        if(!jointZeroed && position > jointEncoder.getPosition())
        return;
        
        // Clamp the position to the min and max
        position = Math.max(getJointMinPosition(), Math.min(getJointMaxPosition(), position));
        jointCommandedPosition = position;
        jointPidController.setReference(position, ControlType.kPosition);
    }
    
    public void setElevatorPosition(double position) {
        
        // Not zeroed and moving away from limit switch
        if(!elevatorZeroed && position > elevatorEncoder.getPosition())
        return;
        
        // Clamp the position to the min and max
        position = Math.max(getElevatorMinPosition(), Math.min(getElevatorMaxPosition(), position));
        elevatorCommandedPosition = position;
        elevatorPidController.setReference(position, ControlType.kPosition);
    }
    
    public void holdJointPosition() {
        if (atJointMaxLimit || atJointMinLimit)
        return;
        setJointPosition(jointEncoder.getPosition());
    }
    
    public void holdElevatorPosition() {
        if (atElevatorMaxLimit || atElevatorMinLimit)
        return;
        setElevatorPosition(elevatorEncoder.getPosition());
    }
    
    public void setTurbotakeAngle(Rotation2d angle) {
        final double conversionFactor = 1.0 / JOINT_ANGLE_CONVERSION_FACTOR; // joint radians -> motor rotations
        final double zeroAngle = -Math.toRadians(15); // angle of joint when the encoder reads zero
        
        double armAngle = Math.toRadians(65) - angle.getRadians();
        
        setJointPosition( (armAngle - zeroAngle) * conversionFactor );
    }
    
    // Commands

    public Command jointZeroPositionCommand() {
        return this.runOnce(() -> setJointPosition(0));
    }
    
    public Command jointStowPositionCommand() {
        return this.runOnce(() -> setJointPosition(getJointStowPosition()));
    }
    
    public Command jointAmpPositionCommand() {
        return this.runOnce(() -> setJointPosition(getJointAmpPosition()));
    }
    
    public Command jointSpeakerPositionCommand() {
        return this.runOnce(() -> setJointPosition(getJointSpeakerPosition()));
    }
    
    public Command jointPositionCommand(double position) {
        return this.runOnce(() -> setJointPosition(position));
    }
    
    public Command jointTrapPositionCommand( ) {
        return this.runOnce(() -> setJointPosition(getJointTrapPosition()));
    }
    
    public Command jointIntakePositionCommand() {
        return this.runOnce(() -> setJointPosition(getJointIntakePosition()));
    }
    
    public Command jointPreIntakePositionCommand() {
        return this.runOnce(() -> setJointPosition(getJointPreIntakePosition()));
    }
    
    public Command elevatorPositionCommand(double position) {
        return this.runOnce(() -> setElevatorPosition(position));
    }
    
    public Command elevatorZeroPositionCommand() {
        return this.runOnce(() -> setElevatorPosition(0));
    }
    
    public Command elevatorStowPositionCommand() {
        return this.runOnce(() -> setElevatorPosition(getElevatorStowPosition()));
    }
    
    public Command elevatorAmpPositionCommand() {
        return this.runOnce(() -> setElevatorPosition(getElevatorAmpPosition()));
    }
    
    public Command elevatorSpeakerPositionCommand() {
        return this.runOnce(() -> setElevatorPosition(getElevatorSpeakerPosition()));
    }
    
    public Command elevatorTrapPositionCommand( ) {
        return this.runOnce(() -> setElevatorPosition(getElevatorTrapPosition()));
    }
    
    public Command elevatorIntakePositionCommand() {
        return this.runOnce(() -> setElevatorPosition(getElevatorIntakePosition()));
    }
    
    public Command elevatorZeroCommand() {
        // use .set instead of setElevatorPercent so that the limit don't apply
        return Commands.sequence(
            jointStowPositionCommand(),
            Commands.waitUntil(() -> (isJointAtPosition(getJointStowPosition(), 1)||atJointMiddleLimitSwitch)),
            Commands.runEnd(() -> elevatorMotor.set(-0.3), () -> elevatorMotor.set(0), this).until(() -> (atElevatorLimitSwitch))
        );
    }
    
    public Command jointZeroCommand() {
        // use .set instead of setJointPercent so that the limit don't apply
        return Commands.runEnd(() -> jointMotor.set(-0.3), () -> jointMotor.set(0), this).until(() -> (atJointMiddleLimitSwitch || atJointBottomLimitSwitch));
    }
    
    // Getters
    
    public boolean isJointAtPosition(double position, double tolerance) {
        return Math.abs(jointEncoder.getPosition() - position) < tolerance;
    }
    
    public boolean isTurbotakeAtAngle(Rotation2d angle, double tolerance) {
        return isJointAtPosition(angle.getRadians() * 1.0 / JOINT_ANGLE_CONVERSION_FACTOR, tolerance);
    }
    
    public boolean isElevatorAtPosition(double position, double tolerance) {
        return Math.abs(elevatorEncoder.getPosition() - position) < tolerance;
    }
    
    public boolean isJointBottomLimitSwitchTriggered() {
        return !(jointBottomReverseSwitch.get());
    }
    
    public boolean isJointMiddleLimitSwitchTriggered() {
        return !(jointMiddleReverseSwitch.get());
    }
    
    public boolean isElevatorLimitSwitchTriggered() {
        return !(elevatorZeroLimitSwitch.get());
    }
    
    public double getJointPosition() {
        return jointEncoder.getPosition();
    }
    
    public double getElevatorPosition() {
        return elevatorEncoder.getPosition();
    }
    
    public boolean isJointZeroed() {
        return jointZeroed;
    }
    
    public boolean isElevatorZeroed() {
        return elevatorZeroed;
    }
    
    // Getters for all preset positions that will return the current value from network tables or the value from Constants if none is published

    public double getJointMinPosition() {
        return !TUNING_MODE ? JOINT_MIN_POSITION : jointMinPositionEntry.get(JOINT_MIN_POSITION);
    }
    
    public double getJointMaxPosition() {
        return !TUNING_MODE ? JOINT_MAX_POSITION : jointMaxPositionEntry.get(JOINT_MAX_POSITION);
    }
    
    public double getJointStowPosition() {
        return !TUNING_MODE ? JOINT_STOW_POSITION : jointStowPositionEntry.get(JOINT_STOW_POSITION);
    }
    
    public double getJointAmpPosition() {
        return !TUNING_MODE ? JOINT_AMP_POSITION : jointAmpPositionEntry.get(JOINT_AMP_POSITION);
    }
    
    public double getJointSpeakerPosition() {
        return !TUNING_MODE ? JOINT_SPEAKER_POSITION : jointSpeakerPositionEntry.get(JOINT_SPEAKER_POSITION);
    }
    
    public double getJointTrapPosition() {
        return !TUNING_MODE ? JOINT_TRAP_POSITION : jointTrapPositionEntry.get(JOINT_TRAP_POSITION);
    }
    
    public double getJointIntakePosition() {
        return !TUNING_MODE ? JOINT_INTAKE_POSITION : jointIntakePositionEntry.get(JOINT_INTAKE_POSITION);
    }
    
    public double getJointPreIntakePosition() {
        return !TUNING_MODE ? JOINT_PRE_INTAKE_POSITION : jointPreIntakePositionEntry.get(JOINT_PRE_INTAKE_POSITION);
    }
    
    public double getElevatorMinPosition() {
        return !TUNING_MODE ? ELEVATOR_MIN_POSITION : elevatorMinPositionEntry.get(ELEVATOR_MIN_POSITION);
    }
    
    public double getElevatorMaxPosition() {
        return !TUNING_MODE ? ELEVATOR_MAX_POSITION : elevatorMaxPositionEntry.get(ELEVATOR_MAX_POSITION);
    }
    
    public double getElevatorStowPosition() {
        return !TUNING_MODE ? ELEVATOR_STOW_POSITION : elevatorStowPositionEntry.get(ELEVATOR_STOW_POSITION);
    }
    
    public double getElevatorAmpPosition() {
        return !TUNING_MODE ? ELEVATOR_AMP_POSITION : elevatorAmpPositionEntry.get(ELEVATOR_AMP_POSITION);
    }
    
    public double getElevatorSpeakerPosition() {
        return !TUNING_MODE ? ELEVATOR_SPEAKER_POSITION : elevatorSpeakerPositionEntry.get(ELEVATOR_SPEAKER_POSITION);
    }
    
    public double getElevatorTrapPosition() {
        return !TUNING_MODE ? ELEVATOR_TRAP_POSITION : elevatorTrapPositionEntry.get(ELEVATOR_TRAP_POSITION);
    }
    
    public double getElevatorIntakePosition() {
        return !TUNING_MODE ? ELEVATOR_INTAKE_POSITION : elevatorIntakePositionEntry.get(ELEVATOR_INTAKE_POSITION);
    }
}