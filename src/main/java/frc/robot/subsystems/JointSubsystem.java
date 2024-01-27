// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.*;

public class JointSubsystem extends SubsystemBase {
  
  // 9:1 reduction
  // 5.53 in circumference

  private static final double ELEVATOR_INCHES_PER_REVOLUTION = 3.53D/9.0D;

  private final CANSparkMax elevator_motor;
  private final DigitalInput elevator_zeroLimitSwitch;
  private final SparkPIDController elevator_pidController;
  private final RelativeEncoder elevator_encoder;

  private double elevator_p = 1.0;
  private double elevator_i = 1.0e-6;
  private double elevator_d = 0.7;

  private double elevator_smartMotionMaxVel = 2000;
  private double elevator_smartMotionMaxAccel = 1000;
  private double elevator_smartMotionMinVel = 0;
  private double elevator_smartMotionAllowedClosedLoopError = 1;

  private int elevator_holdingCurrentLimit = 30;
  private int elevator_runningCurrentLimit = 60;

  private double elevator_commandedPosition = 0;

  private double joint_angleRadians = 0.0;
  private double elevator_extension = 20.0;

  //var
  private final CANSparkMax joint_motor;
  private final SparkPIDController joint_pidController;
  private final RelativeEncoder joint_encoder;
  private final DigitalInput joint_reverseSwitch;

  private double joint_p = 0.075;
  private double joint_i = 0.000005;
  private double joint_d = 0.01;
  private double joint_maxIAccum = 0;
  
  private final ShuffleboardTab shuffleboardTab;

  //limit
  private boolean atLimitSwitch = false;
  private boolean atMaxLimit = false;
  private boolean atMinLimit = false;

  private boolean useLimits = false;

  private boolean zeroed = true;

  public JointSubsystem() {
    joint_motor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
    joint_motor.restoreFactoryDefaults();
    joint_motor.setInverted(false);
    joint_motor.setSmartCurrentLimit(50);

    joint_reverseSwitch = new DigitalInput(JOINT_ZERO_SWITCH_CHANNEL);
    
    joint_encoder = joint_motor.getEncoder();
    joint_encoder.setPosition(0);

    joint_pidController = joint_motor.getPIDController();
    joint_pidController.setP(joint_p);
    joint_pidController.setI(joint_i);
    joint_pidController.setD(joint_d);
    joint_pidController.setIMaxAccum(joint_maxIAccum, 0);
    joint_pidController.setReference(0, ControlType.kDutyCycle);

    elevator_zeroLimitSwitch = new DigitalInput(ELEVATOR_ZERO_SWITCH_CHANNEL);

    elevator_motor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    elevator_motor.restoreFactoryDefaults();
    elevator_motor.setInverted(false);
    elevator_motor.setSmartCurrentLimit(elevator_holdingCurrentLimit, elevator_runningCurrentLimit);

    elevator_encoder = elevator_motor.getEncoder();
    elevator_encoder.setPositionConversionFactor(ELEVATOR_INCHES_PER_REVOLUTION);

    elevator_pidController = elevator_motor.getPIDController();
    elevator_pidController.setP(elevator_p);
    elevator_pidController.setI(elevator_i);
    elevator_pidController.setD(elevator_d);
    elevator_pidController.setSmartMotionMaxVelocity(elevator_smartMotionMaxVel,0);
    elevator_pidController.setSmartMotionMaxAccel(elevator_smartMotionMaxAccel, 0);
    elevator_pidController.setSmartMotionMinOutputVelocity(elevator_smartMotionMinVel,0);
    elevator_pidController.setSmartMotionAllowedClosedLoopError(elevator_smartMotionAllowedClosedLoopError, 0);
    elevator_pidController.setOutputRange(-1.0, 1.0);

    shuffleboardTab = Shuffleboard.getTab("Joint");

    shuffleboardTab.addDouble("Motor Velocity", () -> joint_encoder.getVelocity());
    shuffleboardTab.addDouble("Motor Current", () -> joint_motor.getOutputCurrent());
    shuffleboardTab.addDouble("Motor Position", () -> joint_encoder.getPosition());

    ShuffleboardLayout motorList = shuffleboardTab.getLayout("Motor", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
    motorList.addDouble("Motor RPM", () -> elevator_encoder.getVelocity());
    motorList.addDouble("Motor Current", () -> elevator_motor.getOutputCurrent());
    motorList.addDouble("Motor %", () -> elevator_motor.get());
    motorList.addDouble("Motor Position", () -> elevator_encoder.getPosition());

    ShuffleboardLayout controlList = shuffleboardTab.getLayout("Control", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    controlList.addBoolean("Limit Switch", () -> !elevator_zeroLimitSwitch.get());
    controlList.addBoolean("Zeroed", () -> (zeroed));
    controlList.addBoolean("At Zero", () -> (atLimitSwitch));
    controlList.addBoolean("At Min", () -> (atMinLimit));
    controlList.addBoolean("At Max", () -> (atMaxLimit));
    controlList.addDouble("Commanded Position", () -> (elevator_commandedPosition));

    }

  public void teleopInit() {
      joint_pidController.setReference(0, ControlType.kDutyCycle);
  } 

  public void checkLimitSwitch() {
    if(joint_reverseSwitch.get() || elevator_zeroLimitSwitch.get()) {
        if(atLimitSwitch) {
            // reset encoder on falling edge incase the robot started up and the switch was pressed
            joint_encoder.setPosition(0);
            elevator_encoder.setPosition(0);
        }
        atLimitSwitch = false;
        return;
    }

    zeroed = true;
    if(!atLimitSwitch) {
        //stop motor and reset encoder on rising edge
        atLimitSwitch = true;
        joint_encoder.setPosition(0);
        setPosition(0, true);
    }
    
  }

  public void checkMinLimit() {
    if(joint_encoder.getPosition() > JOINT_MIN_POSITION || elevator_encoder.getPosition() > ELEVATOR_MIN_POSITION) {
        atMinLimit = false;
        return;
    }

    if(!atMinLimit) {
        atMinLimit = true;
        setPosition(JOINT_MIN_POSITION, true);
    }
  }

  public void checkMaxLimit() {
    if(joint_encoder.getPosition() < JOINT_MAX_POSITION || elevator_encoder.getPosition() < ELEVATOR_MAX_POSITION) {
        atMaxLimit = false;
        return;
    }
            
    if(!atMaxLimit) {
        //stop motor on rising edge
        atMaxLimit = true;
        setPosition(JOINT_MAX_POSITION, true);
    }
  }

  public void setMotorPercent(double percent) {
    //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
    if(atMinLimit && percent < 0 && zeroed && useLimits)
        return;
    
    //at max or not yet zeroed and attempting to increase
    if((atMaxLimit || !zeroed) && percent > 0 && useLimits)
        return;
    joint_pidController.setReference(percent * 0.5, ControlType.kDutyCycle);
  }

  public void checkCloseToEnds() {
    if(ELEVATOR_MAX_POSITION - elevator_encoder.getPosition() < ELEVATOR_BUFFER_DISTANCE) {
        elevator_pidController.setOutputRange(-1.0, 0.8);
    }else if(elevator_encoder.getPosition() - ELEVATOR_MIN_POSITION < ELEVATOR_BUFFER_DISTANCE) {
        elevator_pidController.setOutputRange(-0.8, 1.0);
    }else {
        elevator_pidController.setOutputRange(-1.0, 1.0);
    }
  }

  public void checkExtensionPerimeter() {
      //Trig calculation that find extension (extension*sin(angle) & extension*cos(angle))
      double verticalLeg = elevator_extension * Math.sin(joint_angleRadians);
      double horizontalLeg = elevator_extension * Math.cos(joint_angleRadians);

      //Calculation extension from frame perimeter
      if (verticalLeg + JOINT_POSITION_FROM_ROBOT_FLOOR > 47) {
          jointZeroCommand();
          elevatorZeroCommand();
      } else {
        if (Math.toDegrees(joint_angleRadians) > 90 && Math.toDegrees(joint_angleRadians) < 270) {
          if (horizontalLeg + JOINT_POSITION_FROM_ROBOT_BACK > 11.0)
            jointZeroCommand();
            elevatorZeroCommand();
        } else {
          if (horizontalLeg - JOINT_POSITION_FROM_ROBOT_FRONT > 11.0)
            jointZeroCommand();
            elevatorZeroCommand();
        }
      }
  }

  public void setPosition(double position, Boolean joint) {
    //position out of bounds
    //if(position < JOINT_MIN_POSITION || position > JOINT_MAX_POSITION)
    //    return;
    
    //not zeroed and moving away from limit switch
    if (joint) {
      if(!zeroed & position > joint_encoder.getPosition())
          return;

      joint_pidController.setReference(position, ControlType.kPosition);
    } else {
      // Position out of bounds
      if(position < ELEVATOR_MIN_POSITION || position > ELEVATOR_MAX_POSITION)
        return;

      // Not zeroed and moving away from limit switch
      if(!zeroed && position > elevator_encoder.getPosition())
          return;

      elevator_commandedPosition = position;
    }
  }

  public void gotoSetPosition(int positionId) {
    setPosition(JOINT_POSITIONS_ORDERED[positionId], true);
    setPosition(ELEVATOR_POSITIONS_ORDERED[positionId], false);
  }

//set motor positions
  public Command jointStowPositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_STOW_POSITION, true));
  }

  public Command jointAmpPositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_AMP_POSITION, true));
  }

  public Command jointSpeakerPositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_SPEAKER_POSITION, true));
  }

  public Command jointTrapPositionCommand( ) {
    return this.runOnce(() -> setPosition(JOINT_TRAP_POSITION, true));
  }

  public Command jointIntakePositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_INTAKE_POSITION, true));
  }

  public Command elevatorStowPositionCommand() {
    return this.runOnce(() -> setPosition(ELEVATOR_STOW_POSITION, false));
  }

  public Command elevatorAmpPositionCommand() {
    return this.runOnce(() -> setPosition(ELEVATOR_AMP_POSITION, false));
  }

  public Command elevatorSpeakerPositionCommand() {
    return this.runOnce(() -> setPosition(ELEVATOR_SPEAKER_POSITION, false));
  }

  public Command elevatorTrapPositionCommand( ) {
    return this.runOnce(() -> setPosition(ELEVATOR_TRAP_POSITION, false));
  }

  public Command elevatorIntakePositionCommand() {
    return this.runOnce(() -> setPosition(ELEVATOR_INTAKE_POSITION, false));
  }

  public Command jointZeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setMotorPercent(-0.3), this),
        Commands.waitUntil(() -> (atLimitSwitch))
    );
  }

    public Command elevatorZeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setMotorPercent(-0.3), this),
        Commands.waitUntil(() -> (atLimitSwitch))
    );
  }

  public Command disableLimitsCommand() {
      return this.runOnce(() -> useLimits = false);
  }

  public Command enableLimitsCommand() {
      return this.runOnce(() -> useLimits = true);
  }

  @Override
  public void periodic() {
      checkLimitSwitch();
      checkMinLimit();
      checkMaxLimit();
      checkCloseToEnds();
      checkExtensionPerimeter();
      //checkLimitSwitch();
  }

  //Necessary Functions
  public void setJointPID(double p, double i, double d) {
      this.joint_p = p;
      this.joint_i = i;
      this.joint_d = d;

      joint_pidController.setP(p);
      joint_pidController.setI(i);
      joint_pidController.setD(d);
  }

  public void setElevatorPID(double p, double i, double d) {
    this.elevator_p = p;
    this.elevator_i = i;
    this.elevator_d = d;

    elevator_pidController.setP(p);
    elevator_pidController.setI(i);
    elevator_pidController.setD(d);
  }

  public double getJointPosition() {
      return joint_encoder.getPosition();
  }

  public double getElevatorPosition() {
      return elevator_encoder.getPosition();
  }

  public boolean isZeroed() {
      return zeroed;
  }

  public ShuffleboardTab getTab() {
      return shuffleboardTab;
  }
}