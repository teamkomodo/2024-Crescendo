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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import static frc.robot.Constants.*;

public class JointSubsystem extends SubsystemBase {
  
  //var
  private final CANSparkMax motor;
  private final SparkPIDController pidController;
  private final RelativeEncoder encoder;
  private final DigitalInput reverseSwitch;
  
  private final ShuffleboardTab shuffleboardTab;

  private double p = 0.075;
  private double i = 0.000005;
  private double d = 0.01;
  private double maxIAccum = 0;
  
  //limit
  private boolean atLimitSwitch = false;
  private boolean atMaxLimit = false;
  private boolean atMinLimit = false;

  private double commandedPosition = 0;

  private boolean useLimits = false;

  private boolean zeroed = true;

  public JointSubsystem() {
    motor = new CANSparkMax(JOINT_MOTOR_ID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setSmartCurrentLimit(50);

        reverseSwitch = new DigitalInput(JOINT_ZERO_SWITCH_CHANNEL);
        
        encoder = motor.getEncoder();
        encoder.setPosition(0);

        pidController = motor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setIMaxAccum(maxIAccum, 0);
        pidController.setReference(0, ControlType.kDutyCycle);

        shuffleboardTab = Shuffleboard.getTab("Joint");

        shuffleboardTab.addDouble("Motor Velocity", () -> encoder.getVelocity());
        shuffleboardTab.addDouble("Motor Current", () -> motor.getOutputCurrent());
        shuffleboardTab.addDouble("Motor Position", () -> encoder.getPosition());
    }

  public void teleopInit() {
      pidController.setReference(0, ControlType.kDutyCycle);
  } 

  public void checkLimitSwitch() {
    if(reverseSwitch.get()) {
        if(atLimitSwitch) {
            // reset encoder on falling edge incase the robot started up and the switch was pressed
            encoder.setPosition(0);
        }
        atLimitSwitch = false;
        return;
    }

    zeroed = true;
    if(!atLimitSwitch) {
        //stop motor and reset encoder on rising edge
        atLimitSwitch = true;
        encoder.setPosition(0);
        setPosition(0);
    }
    
  }

  public void checkMinLimit() {
    if(encoder.getPosition() > JOINT_MIN_POSITION) {
        atMinLimit = false;
        return;
    }

    if(!atMinLimit) {
        atMinLimit = true;
        setPosition(JOINT_MIN_POSITION);
    }
  }

  public void checkMaxLimit() {
    if(encoder.getPosition() < JOINT_MAX_POSITION) {
        atMaxLimit = false;
        return;
    }
            
    if(!atMaxLimit) {
        //stop motor on rising edge
        atMaxLimit = true;
        setPosition(JOINT_MAX_POSITION);
    }
  }

  public void setMotorPercent(double percent) {
    //at min and attempting to decrease and zeroed (allow movement past limit if not yet zeroed)
    if(atMinLimit && percent < 0 && zeroed && useLimits)
        return;
    
    //at max or not yet zeroed and attempting to increase
    if((atMaxLimit || !zeroed) && percent > 0 && useLimits)
        return;
    pidController.setReference(percent * 0.5, ControlType.kDutyCycle);
  }

  public void setPosition(double position) {
    //position out of bounds
    //if(position < JOINT_MIN_POSITION || position > JOINT_MAX_POSITION)
    //    return;
    
    //not zeroed and moving away from limit switch
    if(!zeroed & position > encoder.getPosition())
        return;

    pidController.setReference(position, ControlType.kPosition);
    commandedPosition = position;
  }

  public void gotoSetPosition(int positionId) {
    setPosition(JOINT_POSITIONS_ORDERED[positionId]);
  }

//set motor positions
  public Command stowPositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_STOW_POSITION));
  }

  public Command ampPositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_AMP_POSITION));
  }

  public Command speakerPositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_SPEAKER_POSITION));
  }

  public Command trapPositionCommand( ) {
    return this.runOnce(() -> setPosition(JOINT_TRAP_POSITION));
  }

  public Command intakePositionCommand() {
    return this.runOnce(() -> setPosition(JOINT_INTAKE_POSITION));
  }

  public Command zeroCommand() {
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
    checkMinLimit();
    checkMaxLimit();
    //checkLimitSwitch();
}

public void setPID(double p, double i, double d) {
    this.p = p;
    this.i = i;
    this.d = d;

    pidController.setP(p);
    pidController.setI(i);
    pidController.setD(d);
}

public double getPosition() {
    return encoder.getPosition();
}

public boolean isZeroed() {
    return zeroed;
}

  public ShuffleboardTab getTab() {
      return shuffleboardTab;
  }
}