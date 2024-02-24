// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.positions.AmpPositionCommand;
import frc.robot.commands.positions.IntakePositionCommand;
import frc.robot.commands.positions.SpeakerPositionCommand;
import frc.robot.commands.positions.StowPositionCommand;
import frc.robot.commands.positions.TrapPositionCommand;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Subsystems

    private final ArmSubsystem armSubsystem = new ArmSubsystem();

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    

    public RobotContainer() {
        //NamedCommands.registerCommand("ExampleCommand", null);

        configureBindings();

    }
    
    private void configureBindings() {

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
        bButton.onTrue(new IntakePositionCommand(armSubsystem));
        xButton.onTrue(new SpeakerPositionCommand(armSubsystem));
        yButton.onTrue(new StowPositionCommand(armSubsystem));
        rightTrigger.onTrue(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(0.5)));
        rightTrigger.onFalse(Commands.runOnce(() -> {armSubsystem.setJointMotorPercent(0); armSubsystem.holdJointPosition();}));
        leftTrigger.onTrue(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(-0.5)));
        leftTrigger.onFalse(Commands.runOnce(() -> {armSubsystem.setJointMotorPercent(0); armSubsystem.holdJointPosition();}));
        rightBumper.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0.5)));
        rightBumper.onFalse(Commands.runOnce(() -> {armSubsystem.setElevatorMotorPercent(0); armSubsystem.holdElevatorPosition();}));
        leftBumper.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(-0.5)));
        leftBumper.onFalse(Commands.runOnce(() -> {armSubsystem.setElevatorMotorPercent(0); armSubsystem.holdElevatorPosition();}));
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
