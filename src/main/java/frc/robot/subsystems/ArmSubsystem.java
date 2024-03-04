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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {
  
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
  private boolean atElevatorLimitSwitchAtLastCheck = false;
  private boolean elevatorZeroed = false;

  private boolean atElevatorMaxLimit = false;
  private boolean atElevatorMinLimit = false;

  // trigonometry
  private double jointAngleRadians = 0.0;
  private double elevatorExtension = 20.0;

  private final BooleanPublisher elevatorZeroedPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("elevatorzeroed").publish();
  private final DoublePublisher elevatorPositionPublisher = NetworkTableInstance.getDefault().getTable("arm").getDoubleTopic("elevatortposition").publish();
  private final BooleanPublisher elevatorSwitchPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("elevatorswitch").publish();
  private final BooleanPublisher atElevatorMinPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("atelevatormin").publish();
  private final BooleanPublisher atElevatorMaxPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("atelevatormax").publish();

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

  double jointVerticalPosition = elevatorExtension * Math.sin(jointAngleRadians);
  double jointHorizontalPosition = Math.abs(elevatorExtension * Math.cos(jointAngleRadians));

  private boolean jointZeroed = false;

  private boolean atJointMiddleLimitSwitch = false;
  private boolean atJointMiddleLimitSwitchAtLastCheck = true;

  private boolean atJointBottomLimitSwitch = false;
  private boolean atJointBottomLimitSwitchAtLastCheck = true;

  private boolean atJointMaxLimit = false;
  private boolean atJointMinLimit = false;

  private String commandedPosition = "stow";

  private final BooleanPublisher jointZeroedPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("jointzeroed").publish();
  private final DoublePublisher jointPositionPublisher = NetworkTableInstance.getDefault().getTable("arm").getDoubleTopic("jointposition").publish();
  private final BooleanPublisher jointMiddleSwitchPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("jointmiddleswitch").publish();
  private final BooleanPublisher jointBottomSwitchPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("jointbottomswitch").publish();
  private final BooleanPublisher atJointMinPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("atjointmin").publish();
  private final BooleanPublisher atJointMaxPublisher = NetworkTableInstance.getDefault().getTable("arm").getBooleanTopic("atjointmax").publish();

  private boolean useLimits = false;

  public ArmSubsystem() {
    jointMotor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
    jointMotor.setInverted(false);
    jointMotor.setSmartCurrentLimit(50);

    jointMiddleReverseSwitch = new DigitalInput(JOINT_MIDDLE_ZERO_SWITCH_CHANNEL);
    jointBottomReverseSwitch = new DigitalInput(JOINT_BOTTOM_ZERO_SWITCH_CHANNEL);
    
    jointEncoder = jointMotor.getEncoder();
    jointEncoder.setPosition(2.6);

    jointPidController = jointMotor.getPIDController();
    jointPidController.setP(jointP);
    jointPidController.setI(jointI);
    jointPidController.setD(jointD);
    jointPidController.setIMaxAccum(jointMaxIAccum, 0);
    jointPidController.setReference(0, ControlType.kPosition);

    jointSecondMotor = new CANSparkMax(JOINT_SECOND_MOTOR_ID, MotorType.kBrushless);
    jointSecondMotor.setInverted(true);
    jointSecondMotor.follow(jointMotor);

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

    }

  public void teleopInit() {

  } 

  public void checkLimitSwitch() {
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

  public void checkMinLimit() {
    //Rising edge
    if(!atJointMinLimit && jointEncoder.getPosition() < JOINT_MIN_POSITION && jointZeroed) { //Rising edge
        atJointMinLimit = true;
        setJointPosition(JOINT_MIN_POSITION);
    } else if(jointEncoder.getPosition() > JOINT_MIN_POSITION) {
        atJointMinLimit = false;
    }

    if(!atElevatorMinLimit && elevatorEncoder.getPosition() < ELEVATOR_MIN_POSITION && elevatorZeroed) { //Rising edge
        atElevatorMinLimit = true;
        setElevatorPosition(ELEVATOR_MIN_POSITION);
    } else if(elevatorEncoder.getPosition() > ELEVATOR_MIN_POSITION) {
        atElevatorMinLimit = false;
    }
  }

  public void checkMaxLimit() {

    //Rising edge
    if(!atJointMaxLimit && jointEncoder.getPosition() > JOINT_MAX_POSITION && jointZeroed) {
      atJointMaxLimit = true;
      setJointPosition(JOINT_MAX_POSITION);
    } else if(jointEncoder.getPosition() < JOINT_MAX_POSITION) {
      atJointMaxLimit = false;
    }

    if(!atElevatorMaxLimit && elevatorEncoder.getPosition() > ELEVATOR_MAX_POSITION && elevatorZeroed) { //Rising edge
        atElevatorMaxLimit = true;
        setElevatorPosition(ELEVATOR_MAX_POSITION);
    } else if(elevatorEncoder.getPosition() < ELEVATOR_MAX_POSITION) {
        atElevatorMaxLimit = false;
    }
  }

  public void setJointMotorPercent(double percent) {
    //if the speed is negative, we cannot move if we're at the min limit AND the joint is zeroed
    if(percent < 0 && atJointMinLimit && jointZeroed)
        return;
    
    // if the speed is positive, we cannot move if we're at the max limit OR the joint is not zeroed
    if(percent > 0 && (atJointMaxLimit || !jointZeroed))
        return;
      
    jointPidController.setReference(percent * 0.5, ControlType.kDutyCycle);
  }

  public void setElevatorMotorPercent(double percent) {
    //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
      if(atElevatorMinLimit && percent < 0 && elevatorZeroed)
          return;
      
      //at max or not yet zeroed and attempting to increase
      if((atElevatorMaxLimit || !elevatorZeroed) && percent > 0)
          return;

      elevatorPidController.setReference(percent * 0.5, ControlType.kDutyCycle);
  }

  public void checkExtensionPerimeter() {
      //Calculation extension from frame perimeter
      if (jointVerticalPosition + JOINT_POSITION_FROM_FLOOR > VERTICAL_EXTENSION_LIMIT) {
          elevatorZeroCommand();
      } else {
        if (Math.toDegrees(jointAngleRadians) < JOINT_VERTICAL_ANGLE) {
          if (jointHorizontalPosition - JOINT_POSITION_FROM_ROBOT_FRONT > HORIZONTAL_EXTENSION_LIMIT)
            elevatorZeroCommand();
        } else {
          if (jointHorizontalPosition - JOINT_POSITION_FROM_ROBOT_BACK > HORIZONTAL_EXTENSION_LIMIT)
            elevatorZeroCommand();
        }
      }
  }

  public void updateArmVariables() {
      elevatorExtension = getElevatorPosition() * ELEVATOR_INCHES_PER_REVOLUTION;
      //Trig calculation that find extension (extension*sin(angle) & extension*cos(angle))
      jointVerticalPosition = elevatorExtension * Math.sin(jointAngleRadians);
      jointHorizontalPosition = Math.abs(elevatorExtension * Math.cos(jointAngleRadians));
  }

  public void setJointPosition(double position) {
    
    //not zeroed and moving away from limit switch
    if(!jointZeroed && position > jointEncoder.getPosition())
        return;

    if(position < JOINT_MIN_POSITION)
        jointPidController.setReference(JOINT_MIN_POSITION, ControlType.kPosition);
    else if (position > JOINT_MAX_POSITION)
      jointPidController.setReference(JOINT_MAX_POSITION, ControlType.kPosition);
    else
      jointPidController.setReference(position, ControlType.kPosition);

      // Position out of bounds
  }

public void setElevatorPosition(double position) {
  //Not zeroed and moving away from limit switch
  if(!elevatorZeroed && position > elevatorEncoder.getPosition())
      return;

      // Position out of bounds
  if(position < ELEVATOR_MIN_POSITION)
    elevatorPidController.setReference(ELEVATOR_MIN_POSITION, ControlType.kPosition);
  else if (position > ELEVATOR_MAX_POSITION)
    elevatorPidController.setReference(ELEVATOR_MAX_POSITION, ControlType.kPosition);
  else
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

  public void gotoSetPosition(int positionId) {
    setJointPosition(JOINT_POSITIONS_ORDERED[positionId]);
    setElevatorPosition(ELEVATOR_POSITIONS_ORDERED[positionId]);
  }

  public void setTurbotakeAngle(Rotation2d angle) {
    final double conversionFactor = 1.0 / (JOINT_REDUTION * 2.0 * Math.PI); // joint radians -> motor rotations
    final double zeroAngle = -Math.toRadians(30); // angle of joint when the encoder reads zero

    double armAngle = Math.toRadians(65) - angle.getRadians();

    setJointPosition( (armAngle - zeroAngle) * conversionFactor );
  }

//set motor positions
  public Command jointZeroPositionCommand() {
    return this.runOnce(() -> setJointPosition(0));
  }

  public Command jointStowPositionCommand() {
    commandedPosition = "stow";
    return this.runOnce(() -> setJointPosition(JOINT_STOW_POSITION));
  }

  public Command jointAmpPositionCommand() {
    commandedPosition = "amp";
    return this.runOnce(() -> setJointPosition(JOINT_AMP_POSITION));
  }

  public Command jointSpeakerPositionCommand() {
    commandedPosition = "speaker";
    return this.runOnce(() -> setJointPosition(JOINT_SPEAKER_POSITION));
    // jointAngleRadians = getJointPosition() * JOINT_RADIAN_PER_REVOLUTION;
    // double targetAngle = Math.atan(JOINT_AVERAGE_SHOOT_HEIGHT / robotDistanceFromSpeaker);
    // double jointTargetAngle = targetAngle + TURBOTAKE_JOINT_RADIAN_OFFSET;
    // return this.runOnce(() -> setJointPosition(jointTargetAngle / JOINT_RADIAN_PER_REVOLUTION));
  }

  public Command jointTrapPositionCommand( ) {
    return this.runOnce(() -> setJointPosition(JOINT_TRAP_POSITION));
  }

  public Command jointIntakePositionCommand() {
    commandedPosition = "intake";
    return this.runOnce(() -> setJointPosition(JOINT_INTAKE_POSITION));
  }

  public Command jointPreIntakePositionCommand() {
    return this.runOnce(() -> setJointPosition(JOINT_PRE_INTAKE_POSITION));
  }

  public Command elevatorZeroPositionCommand() {
    return this.runOnce(() -> setElevatorPosition(0));
  }

  public Command elevatorStowPositionCommand() {
    return this.runOnce(() -> setElevatorPosition(ELEVATOR_STOW_POSITION));
  }

  public Command elevatorAmpPositionCommand() {
    return this.runOnce(() -> setElevatorPosition(ELEVATOR_AMP_POSITION));
  }

  public Command elevatorSpeakerPositionCommand() {
    return this.runOnce(() -> setElevatorPosition(ELEVATOR_SPEAKER_POSITION));
  }

  public Command elevatorTrapPositionCommand( ) {
    return this.runOnce(() -> setElevatorPosition(ELEVATOR_TRAP_POSITION));
  }

  public Command elevatorIntakePositionCommand() {
    return this.runOnce(() -> setElevatorPosition(ELEVATOR_INTAKE_POSITION));
  }

  public Command speakerPositionCommand() {
    double a = robotDistanceFromSpeaker;
    double b = robotAverageTurbotakeHeightFromSpeaker;
    double c = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    double turbotakeAngle = Math.acos((Math.pow(c, 2) + Math.pow(a, 2) + Math.pow(b, 2)) / (2 * c * a));
    double jointAngle = Math.toRadians(70) - turbotakeAngle;
    double jointPosition = jointAngle / JOINT_RADIAN_PER_REVOLUTION;
    return this.runOnce(() -> {
      setJointPosition(0);
      setJointPosition(jointPosition);
    });
  }

  public Command elevatorZeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setElevatorMotorPercent(-0.3), this),
        Commands.waitUntil(() -> (atElevatorLimitSwitch))
    );
  }

  public Command jointZeroCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> setJointMotorPercent(-0.3), this),
        Commands.waitUntil(() -> (atJointBottomLimitSwitch || atJointMiddleLimitSwitch))
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
      //updateArmVariables();
      checkLimitSwitch();
      checkMinLimit();
      checkMaxLimit();
      // checkExtensionPerimeter();
      updateTelemetry();
  }

  public void updateTelemetry() {
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

  public String getCommandedPosition() {
    return commandedPosition;
  }
}