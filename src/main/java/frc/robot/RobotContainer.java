// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.positions.SpeakerPositionCommand;
import frc.robot.commands.positions.StowPositionCommand;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TurbotakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

public class RobotContainer {    

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_XBOX_PORT);

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final TurbotakeSubsystem turbotakeSubsystem = new TurbotakeSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    private final TeleopStateMachine teleopStateMachine = new TeleopStateMachine(drivetrainSubsystem, armSubsystem, turbotakeSubsystem, ledSubsystem, driverController.getHID(), operatorController.getHID());

    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        /*
         * Driver Controls
         * 
         * LT - Slow Mode
         * 
         */

        Trigger driverRT = driverController.rightTrigger();

        driverRT.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        driverRT.onFalse(drivetrainSubsystem.disableSlowModeCommand());

        Trigger driverStart = driverController.start();
        driverStart.onTrue(drivetrainSubsystem.zeroGyroCommand());

        // deadband and curves are applied in command
        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> ( -driverController.getLeftY() ), // -Y on left joystick is +X for robot
                () -> ( -driverController.getLeftX() ), // -X on left joystick is +Y for robot
                () -> ( -driverController.getRightX() ) // -X on right joystick is +Z for robot
            )
        );

        driverController.a().whileTrue(drivetrainSubsystem.pointToSpeakerCommand());

        /*
         * Operator Controls
         * 
         * A - Stow
         * X - Command Align Amp
         * 
         * Left Stick X - Elevator
         * Right Stick Y - Joint
         * 
         * Right Trigger - Command Shoot State
         * Left Trigger - Command Intake State
         * 
         * Right Bumper - Command Spin Up State
         * Left Bumper - Command Score Amp
         */

        Trigger operatorA = operatorController.a();
        operatorA.onTrue(new StowPositionCommand(armSubsystem));
        //operatorA.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(0.8), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));

        Trigger operatorX = operatorController.x();
        operatorX.whileTrue(teleopStateMachine.alignAmpCommand());

        Trigger operatorRB = operatorController.rightBumper();
        operatorRB.whileTrue(teleopStateMachine.spinUpCommand());

        Trigger operatorLB = operatorController.leftBumper();
        operatorLB.whileTrue(teleopStateMachine.scoreAmpCommand());
        
        Trigger operatorRT = operatorController.rightTrigger();
        operatorRT.whileTrue(teleopStateMachine.shootSpeakerCommand());

        Trigger operatorLT = operatorController.leftTrigger();
        operatorLT.whileTrue(teleopStateMachine.pickupGroundCommand());
        
        Trigger operatorRSY = new Trigger(() -> (Math.abs(operatorController.getRightY()) > XBOX_DEADBAND));
        operatorRSY.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(-operatorController.getRightY()), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));

        Trigger operatorLSX = new Trigger(() -> (Math.abs(operatorController.getLeftX()) > XBOX_DEADBAND));
        operatorLSX.whileTrue(Commands.runEnd(() -> armSubsystem.setElevatorMotorPercent(operatorController.getLeftX()), () -> armSubsystem.setElevatorPosition(armSubsystem.getElevatorPosition())));
        
        Trigger operatorStart = operatorController.start();
        operatorStart.whileTrue(Commands.runEnd(() -> climberSubsystem.setClimberDutyCycle(0.3), () -> climberSubsystem.holdClimberPosition()));

        Trigger operatorBack = operatorController.back();
        operatorBack.whileTrue(Commands.runEnd(() -> climberSubsystem.setClimberDutyCycle(-0.3), () -> climberSubsystem.holdClimberPosition()));

        armSubsystem.setJointMotorPercent(0);
    }

    public void teleopInit() {
        armSubsystem.teleopInit();
        turbotakeSubsystem.teleopInit();
    }
    
    public Command getAutonomousCommand() {
        //return null;
        return Commands.sequence(
            new SpeakerPositionCommand(armSubsystem),
            Commands.waitSeconds(1),
            turbotakeSubsystem.shootForSpeaker()
        );
    }

    public TeleopStateMachine getTeleopStateMachine() {
        return teleopStateMachine;
    }

}
