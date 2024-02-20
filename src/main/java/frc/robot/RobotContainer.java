// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.commands.positions.AmpPositionCommand;
import frc.robot.commands.positions.IntakePositionCommand;
import frc.robot.commands.positions.SpeakerPositionCommand;
import frc.robot.commands.positions.StowPositionCommand;
import frc.robot.commands.positions.TrapPositionCommand;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Subsystems
    //private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        // Triggers
        Trigger aButton = driverController.a();
        Trigger bButton = driverController.b();
        Trigger xButton = driverController.x();
        Trigger yButton = driverController.y();
        Trigger rightTrigger = driverController.rightTrigger();
        Trigger leftTrigger = driverController.leftTrigger();
        Trigger rightBumper = driverController.rightBumper();
        Trigger leftBumper = driverController.leftBumper();


        // Elevator/joint position commands
        aButton.onTrue(new AmpPositionCommand(armSubsystem));
        bButton.onTrue(new StowPositionCommand(armSubsystem));
        xButton.onTrue(armSubsystem.elevatorAmpPositionCommand());
        yButton.onTrue(armSubsystem.elevatorSpeakerPositionCommand());
        rightTrigger.onTrue(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(0.5)));
        rightTrigger.onFalse(Commands.runOnce(() -> {armSubsystem.setJointMotorPercent(0); armSubsystem.holdJointPosition();}));
        leftTrigger.onTrue(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(-0.5)));
        leftTrigger.onFalse(Commands.runOnce(() -> {armSubsystem.setJointMotorPercent(0); armSubsystem.holdJointPosition();}));
        rightBumper.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0.5)));
        rightBumper.onFalse(Commands.runOnce(() -> {armSubsystem.setElevatorMotorPercent(0); armSubsystem.holdElevatorPosition();}));
        leftBumper.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(-0.5)));
        leftBumper.onFalse(Commands.runOnce(() -> {armSubsystem.setElevatorMotorPercent(0); armSubsystem.holdElevatorPosition();}));

        

        // Slow/Fast Mode
        // rTrigger.onTrue(drivetrainSubsystem.disableSlowModeCommand());
        // rTrigger.onFalse(drivetrainSubsystem.enableSlowModeCommand());

        // // Drivetrain controls
        // drivetrainSubsystem.setDefaultCommand(
        //     drivetrainSubsystem.joystickDriveCommand(
        //         () -> (driverController.getLeftY()), // +Y on left joystick is +X for robot
        //         () -> (driverController.getLeftX()), // +X on left joystick is +Y for robot
        //         () -> (-driverController.getRightX())) // -X on right joystick is +Z for robot
        // );
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
