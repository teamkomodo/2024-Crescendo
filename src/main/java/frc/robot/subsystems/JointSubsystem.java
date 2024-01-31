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

  private double elevatorCommandedPosition = 0;

  private double jointAngleRadians = 0.0;
  private double elevatorExtension = 20.0;

  //var
  private final CANSparkMax jointMotor;
  private final SparkPIDController jointPidController;
  private final RelativeEncoder jointEncoder;
  private final DigitalInput jointReverseSwitch;

  private double jointP = 0.075;
  private double jointI = 0.000005;
  private double jointD = 0.01;
  private double jointMaxIAccum = 0;
  
  private final ShuffleboardTab shuffleboardTab;

  //limit
  private boolean atLimitSwitch = false;
  private boolean atMaxLimit = false;
  private boolean atMinLimit = false;

  private boolean useLimits = false;

  private boolean zeroed = true;

  public JointSubsystem() {
    jointMotor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
    jointMotor.restoreFactoryDefaults();
    jointMotor.setInverted(false);
    jointMotor.setSmartCurrentLimit(50);

    jointReverseSwitch = new DigitalInput(JOINT_ZERO_SWITCH_CHANNEL);
    
    jointEncoder = jointMotor.getEncoder();
    jointEncoder.setPosition(0);

    jointPidController = jointMotor.getPIDController();
    jointPidController.setP(jointP);
    jointPidController.setI(jointI);
    jointPidController.setD(jointD);
    jointPidController.setIMaxAccum(jointMaxIAccum, 0);
    jointPidController.setReference(0, ControlType.kDutyCycle);

    elevatorZeroLimitSwitch = new DigitalInput(ELEVATOR_ZERO_SWITCH_CHANNEL);

    elevatorMotor = new CANSparkMax(ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    elevatorMotor.restoreFactoryDefaults();
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

    shuffleboardTab = Shuffleboard.getTab("Joint");

    shuffleboardTab.addDouble("Motor Velocity", () -> jointEncoder.getVelocity());
    shuffleboardTab.addDouble("Motor Current", () -> jointMotor.getOutputCurrent());
    shuffleboardTab.addDouble("Motor Position", () -> jointEncoder.getPosition());

    ShuffleboardLayout motorList = shuffleboardTab.getLayout("Motor", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 0);
    motorList.addDouble("Motor RPM", () -> elevatorEncoder.getVelocity());
    motorList.addDouble("Motor Current", () -> elevatorMotor.getOutputCurrent());
    motorList.addDouble("Motor %", () -> elevatorMotor.get());
    motorList.addDouble("Motor Position", () -> elevatorEncoder.getPosition());

    ShuffleboardLayout controlList = shuffleboardTab.getLayout("Control", BuiltInLayouts.kList).withSize(2, 4).withPosition(2, 0);
    controlList.addBoolean("Limit Switch", () -> !elevatorZeroLimitSwitch.get());
    controlList.addBoolean("Zeroed", () -> (zeroed));
    controlList.addBoolean("At Zero", () -> (atLimitSwitch));
    controlList.addBoolean("At Min", () -> (atMinLimit));
    controlList.addBoolean("At Max", () -> (atMaxLimit));
    controlList.addDouble("Commanded Position", () -> (elevatorCommandedPosition));

    }

  public void teleopInit() {
      jointPidController.setReference(0, ControlType.kDutyCycle);
  } 

  public void checkLimitSwitch() {
    if(jointReverseSwitch.get() || elevatorZeroLimitSwitch.get()) {
        if(atLimitSwitch) {
            // reset encoder on falling edge incase the robot started up and the switch was pressed
            jointEncoder.setPosition(0);
            elevatorEncoder.setPosition(0);
        }
        atLimitSwitch = false;
        return;
    }

    zeroed = true;
    if(!atLimitSwitch) {
        //stop motor and reset encoder on rising edge
        atLimitSwitch = true;
        jointEncoder.setPosition(0);
        setPosition(0, true);
    }
    
  }

  public void checkMinLimit() {
    if(jointEncoder.getPosition() > JOINT_MIN_POSITION || elevatorEncoder.getPosition() > ELEVATOR_MIN_POSITION) {
        atMinLimit = false;
        return;
    }

    if(!atMinLimit) {
        atMinLimit = true;
        setPosition(JOINT_MIN_POSITION, true);
    }
  }

  public void checkMaxLimit() {
    if(jointEncoder.getPosition() < JOINT_MAX_POSITION || elevatorEncoder.getPosition() < ELEVATOR_MAX_POSITION) {
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
    jointPidController.setReference(percent * 0.5, ControlType.kDutyCycle);
  }

  public void checkCloseToEnds() {
    if(ELEVATOR_MAX_POSITION - elevatorEncoder.getPosition() < ELEVATOR_BUFFER_DISTANCE) {
        elevatorPidController.setOutputRange(-1.0, 0.8);
    }else if(elevatorEncoder.getPosition() - ELEVATOR_MIN_POSITION < ELEVATOR_BUFFER_DISTANCE) {
        elevatorPidController.setOutputRange(-0.8, 1.0);
    }else {
        elevatorPidController.setOutputRange(-1.0, 1.0);
    }
  }

  public void checkExtensionPerimeter() {
      //Trig calculation that find extension (extension*sin(angle) & extension*cos(angle))
      double verticalLeg = elevatorExtension * Math.sin(jointAngleRadians);
      double horizontalLeg = elevatorExtension * Math.cos(jointAngleRadians);

      //Calculation extension from frame perimeter
      if (verticalLeg + JOINT_POSITION_FROM_ROBOT_FLOOR > 47) {
          jointZeroCommand();
          elevatorZeroCommand();
      } else {
        if (Math.toDegrees(jointAngleRadians) > 90 && Math.toDegrees(jointAngleRadians) < 270) {
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
      if(!zeroed & position > jointEncoder.getPosition())
          return;

      jointPidController.setReference(position, ControlType.kPosition);
    } else {
      // Position out of bounds
      if(position < ELEVATOR_MIN_POSITION || position > ELEVATOR_MAX_POSITION)
        return;

      // Not zeroed and moving away from limit switch
      if(!zeroed && position > elevatorEncoder.getPosition())
          return;

      elevatorCommandedPosition = position;
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
      this.jointP = p;
      this.jointI = i;
      this.jointD = d;

      jointPidController.setP(p);
      jointPidController.setI(i);
      jointPidController.setD(d);
  }

  public void setElevatorPID(double p, double i, double d) {
    this.elevatorP = p;
    this.elevatorI = i;
    this.elevatorD = d;

    elevatorPidController.setP(p);
    elevatorPidController.setI(i);
    elevatorPidController.setD(d);
  }

  public double getJointPosition() {
      return jointEncoder.getPosition();
  }

  public double getElevatorPosition() {
      return elevatorEncoder.getPosition();
  }

  public boolean isZeroed() {
      return zeroed;
  }

  public ShuffleboardTab getTab() {
      return shuffleboardTab;
  }
}