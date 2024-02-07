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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.*;

public class JointSubsystem extends SubsystemBase {
  
  //alignment
  private double robotDistanceFromSpeaker = 0;
  private double robotAverageTurbotakeHeightFromSpeaker = 0;

  // 9:1 reduction
  // 5.53 in circumference

  private static final double ELEVATOR_INCHES_PER_REVOLUTION = 0; //3.53D/9.0D

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

  private boolean atElevatorLimitSwitch = false;
  private boolean elevatorZeroed = true;

  private boolean atElevatorMaxLimit = false;
  private boolean atElevatorMinLimit = false;

  // trigonometry
  private double jointAngleRadians = 0.0;
  private double elevatorExtension = 20.0;

  //var
  private final CANSparkMax jointMotor;
  private final SparkPIDController jointPidController;
  private final RelativeEncoder jointEncoder;
  private final DigitalInput jointMiddleReverseSwitch;
  private final DigitalInput jointBottomReverseSwitch;

  private double jointP = 0.075;
  private double jointI = 0.000005;
  private double jointD = 0.01;
  private double jointMaxIAccum = 0;

  double jointVerticalPosition = elevatorExtension * Math.sin(jointAngleRadians);
  double jointHorizontalPosition = Math.abs(elevatorExtension * Math.cos(jointAngleRadians));
  
  private final ShuffleboardTab shuffleboardTab;

  private boolean jointPositiveVelocity = true;

  private boolean atJointLimitSwitch = false;
  private boolean jointZeroed = true;

  private boolean atJointMiddleLimitSwitch = false;
  private boolean atJointMiddleLimitSwitchAtLastCheck = true;

  private boolean atJointBottomLimitSwitch = false;
  private boolean atJointBottomLimitSwitchAtLastCheck = true;
  private boolean jointBottomLimitSwitchOn = true;

  private boolean atJointMaxLimit = false;
  private boolean atJointMinLimit = false;

  private boolean useLimits = false;

  private boolean zeroed = true;

  public JointSubsystem() {
    jointMotor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
    jointMotor.restoreFactoryDefaults();
    jointMotor.setInverted(false);
    jointMotor.setSmartCurrentLimit(50);

    jointMiddleReverseSwitch = new DigitalInput(JOINT_MIDDLE_ZERO_SWITCH_CHANNEL);
    jointBottomReverseSwitch = new DigitalInput(JOINT_BOTTOM_ZERO_SWITCH_CHANNEL);
    
    jointEncoder = jointMotor.getEncoder();
    jointEncoder.setPosition(0);

    jointPidController = jointMotor.getPIDController();
    jointPidController.setP(jointP);
    jointPidController.setI(jointI);
    jointPidController.setD(jointD);
    jointPidController.setIMaxAccum(jointMaxIAccum, 0);
    jointPidController.setReference(0, ControlType.kPosition);

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
    controlList.addBoolean("At Joint Zero", () -> (atJointLimitSwitch));
    controlList.addBoolean("At Elevator Zero", () -> (atElevatorLimitSwitch));
    controlList.addBoolean("At Joint Min", () -> (atJointMinLimit));
    controlList.addBoolean("At Joint Max", () -> (atJointMaxLimit));
    controlList.addBoolean("At Elevator Min", () -> (atElevatorMinLimit));
    controlList.addBoolean("At Elevator Max", () -> (atElevatorMaxLimit));
    controlList.addDouble("Commanded Position", () -> (elevatorCommandedPosition));

    SmartDashboard.putNumber("Joint Motor Velocity", jointEncoder.getVelocity());
    SmartDashboard.putNumber("Joint Motor Current", jointMotor.getOutputCurrent());
    SmartDashboard.putNumber("Joint Motor Position", jointEncoder.getPosition());

    SmartDashboard.putNumber("Elevator Motor RPM", elevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Motor Current", elevatorMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator Motor Percent", elevatorMotor.get());
    SmartDashboard.putNumber("Elevator Motor Position", elevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Commanded Position", elevatorCommandedPosition);

    SmartDashboard.putBoolean("At Elevator Zero", atElevatorLimitSwitch);
    SmartDashboard.putBoolean("At Elevator Min", atElevatorMinLimit);
    SmartDashboard.putBoolean("At Elevator Max", atElevatorMaxLimit);
    SmartDashboard.putBoolean("At Joint Zero", atJointLimitSwitch);
    SmartDashboard.putBoolean("At Joint Min", atJointMinLimit);
    SmartDashboard.putBoolean("At Joint Max", atJointMaxLimit);

    }

  public void teleopInit() {
      jointPidController.setReference(0, ControlType.kDutyCycle);
      jointZeroCommand();
      elevatorZeroCommand();
  } 

  public void checkLimitSwitch() {

    if (!jointMiddleReverseSwitch.get())
      atJointMiddleLimitSwitchAtLastCheck = atJointMiddleLimitSwitch;
      atJointMiddleLimitSwitch = true;
    if (!jointBottomReverseSwitch.get())
      atJointBottomLimitSwitchAtLastCheck = atJointBottomLimitSwitch;
      atJointBottomLimitSwitch = true;

    if (!jointPositiveVelocity) {
      if(atJointMiddleLimitSwitch && !atJointMiddleLimitSwitchAtLastCheck) {
        // reset encoder on falling edge incase the robot started up and the switch was pressed
        jointEncoder.setPosition(0);
        jointZeroed = true;
        jointBottomLimitSwitchOn = false;
      }

      if(atJointBottomLimitSwitch && !atJointBottomLimitSwitchAtLastCheck && jointBottomLimitSwitchOn) {
        // reset encoder on falling edge incase the robot started up and the switch was pressed
        jointEncoder.setPosition(0);
        jointZeroed = true;
        jointMiddleZeroCommand();
      }
    } else {
      if(!atJointMiddleLimitSwitch && atJointMiddleLimitSwitchAtLastCheck) {
        // reset encoder on falling edge incase the robot started up and the switch was pressed
        jointEncoder.setPosition(0);
        jointZeroed = true;
        jointBottomLimitSwitchOn = false;
      }
    }

    if(!elevatorZeroLimitSwitch.get()) {
      // reset encoder on falling edge incase the robot started up and the switch was pressed
      elevatorEncoder.setPosition(0);
      atElevatorLimitSwitch = false;
      elevatorZeroed = true;
    }

    if (jointZeroed && elevatorZeroed) {
      zeroed = true;
    }

    
    if(!atJointLimitSwitch) {
      //stop motor and reset encoder on rising edge
      atJointLimitSwitch = true;
      jointEncoder.setPosition(0);
      setPosition(0, true);
    }
  }

  public void checkMinLimit() {
    if(jointEncoder.getPosition() > JOINT_MIN_POSITION || elevatorEncoder.getPosition() > ELEVATOR_MIN_POSITION) {
        atJointMinLimit = false;
    }

    if(!atJointMinLimit) {
        atJointMinLimit = true;
        setPosition(JOINT_MIN_POSITION, true);
    }

    if(elevatorEncoder.getPosition() > ELEVATOR_MIN_POSITION) {
        atElevatorMinLimit = false;
    }

    if(!atElevatorMinLimit) {
        atElevatorMinLimit = true;
        setPosition(ELEVATOR_MIN_POSITION, true);
    }
  }

  public void checkMaxLimit() {
    if(jointEncoder.getPosition() < JOINT_MAX_POSITION) {
        atJointMaxLimit = false;
    }
            
    if(!atJointMaxLimit) {
        atJointMaxLimit = true;
        setPosition(JOINT_MAX_POSITION, true);
    }

    if(elevatorEncoder.getPosition() < ELEVATOR_MAX_POSITION) {
        atElevatorMaxLimit = false;
    }
            
    if(!atElevatorMaxLimit) {
        atElevatorMaxLimit = true;
        setPosition(ELEVATOR_MAX_POSITION, true);
    }
  }

  public void setMotorPercent(double percent, Boolean joint) {
    if (joint) {
      //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
      if(atJointMinLimit && percent < 0 && jointZeroed && useLimits)
          return;
      
      //at max or not yet zeroed and attempting to increase
      if((atJointMaxLimit || !jointZeroed) && percent > 0 && useLimits)
          return;
      jointPidController.setReference(percent * 0.5, ControlType.kDutyCycle);
    } else {
      //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
      if(atElevatorMinLimit && percent < 0 && elevatorZeroed && useLimits)
          return;
      
      //at max or not yet zeroed and attempting to increase
      if((atElevatorMaxLimit || !elevatorZeroed) && percent > 0 && useLimits)
          return;
      elevatorPidController.setReference(percent * 0.5, ControlType.kDutyCycle);
    }
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
      //Calculation extension from frame perimeter
      if (jointVerticalPosition + JOINT_POSITION_FROM_FLOOR > VERTICAL_EXTENSION_LIMIT) {
          jointZeroCommand(); // Chnage later
          elevatorZeroCommand();
      } else {
        if (Math.toDegrees(jointAngleRadians) < JOINT_VERTICAL_ANGLE) {
          if (jointHorizontalPosition - JOINT_POSITION_FROM_ROBOT_FRONT > HORIZONTAL_EXTENSION_LIMIT)
            jointZeroCommand(); // Change later
            elevatorZeroCommand();
        } else {
          if (jointHorizontalPosition - JOINT_POSITION_FROM_ROBOT_BACK > HORIZONTAL_EXTENSION_LIMIT)
            jointZeroCommand(); // Change later
            elevatorZeroCommand();
        }
      }
  }

  public void updateArmVariables() {
      jointAngleRadians = getJointPosition() * JOINT_RADIAN_PER_REVOLUTION;
      elevatorExtension = getElevatorPosition() * ELEVATOR_INCHES_PER_REVOLUTION;
      //Trig calculation that find extension (extension*sin(angle) & extension*cos(angle))
      jointVerticalPosition = elevatorExtension * Math.sin(jointAngleRadians);
      jointHorizontalPosition = Math.abs(elevatorExtension * Math.cos(jointAngleRadians));

    if (jointEncoder.getVelocity() > 0)
      jointPositiveVelocity = true;
    else
      jointPositiveVelocity = false;
  }

  public void setPosition(double position, Boolean joint) {
    //position out of bounds
    //if(position < JOINT_MIN_POSITION || position > JOINT_MAX_POSITION)
    //    return;
    
    //not zeroed and moving away from limit switch
    if (joint) {
      if(!jointZeroed & position > jointEncoder.getPosition())
          return;

      jointPidController.setReference(position, ControlType.kPosition);
    } else {
      // Position out of bounds
      if(position < ELEVATOR_MIN_POSITION || position > ELEVATOR_MAX_POSITION)
        return;

      // Not zeroed and moving away from limit switch
      if(!elevatorZeroed && position > elevatorEncoder.getPosition())
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

  public Command speakerPositionCommand() {
    double a = robotDistanceFromSpeaker;
    double b = robotAverageTurbotakeHeightFromSpeaker;
    double c = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    double turbotakeAngle = Math.acos((Math.pow(c, 2) + Math.pow(a, 2) + Math.pow(b, 2)) / (2 * c * a));
    double jointAngle = Math.toRadians(70) - turbotakeAngle;
    double jointPosition = jointAngle / JOINT_RADIAN_PER_REVOLUTION;
    return this.runOnce(() -> {
      setPosition(0, false);
      setPosition(jointPosition, true);
    });
  }

  public Command jointZeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setMotorPercent(-0.3, true), this),
        Commands.waitUntil(() -> (atJointMiddleLimitSwitch || atJointBottomLimitSwitch))
    );
  }

  public Command jointMiddleZeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setMotorPercent(0.3, true), this),
        Commands.waitUntil(() -> (atJointMiddleLimitSwitch))
    );
  }

    public Command elevatorZeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setMotorPercent(-0.3, false), this),
        Commands.waitUntil(() -> (atElevatorLimitSwitch))
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
      updateArmVariables();
      checkLimitSwitch();
      checkMinLimit();
      checkMaxLimit();
      checkCloseToEnds();
      checkExtensionPerimeter();
  }

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

  public boolean isJointZeroed() {
      return jointZeroed;
  }

  public boolean isElevatorZeroed() {
      return elevatorZeroed;
  }

  public ShuffleboardTab getTab() {
      return shuffleboardTab;
  }
}