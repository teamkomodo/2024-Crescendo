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

import frc.robot.subsystems.TurbotakeSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

public class RobotContainer {

    // Subsystems
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    public final TurbotakeSubsystem turbotakeSubsystem = new TurbotakeSubsystem();

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        //NamedCommands.registerCommand("ExampleCommand", null);

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser",autoChooser);
    }
    
    private void configureBindings() {
        //testing binds

        //state buttons
        Trigger aButton = driverController.a();//amp
        Trigger bButton = driverController.b();//speaker

        //SysID testing binds
        Trigger rightTrigger = driverController.rightTrigger();

        rightTrigger.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        rightTrigger.onFalse(drivetrainSubsystem.disableSlowModeCommand());

        Trigger startButton = driverController.start();
        startButton.onTrue(drivetrainSubsystem.zeroGyroCommand());

        // deadband and curves are applied in command
        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> ( -driverController.getLeftY() ), // -Y on left joystick is +X for robot
                () -> ( -driverController.getLeftX() ), // -X on left joystick is +Y for robot
                () -> ( -driverController.getRightX() ) // -X on right joystick is +Z for robot
            )
        );

        //motor buttons
        Trigger rightBumper = driverController.rightBumper();
        Trigger leftBumper = driverController.leftBumper();

        //run duty cycles
        aButton.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setIndexerPercent(1), () -> turbotakeSubsystem.setIndexerPercent(0), turbotakeSubsystem));
        bButton.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setShooterPercent(1), () -> turbotakeSubsystem.setShooterPercent(0), turbotakeSubsystem));
        
        //runs the motors directly
        rightBumper.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setIndexerVelocity(1), () -> turbotakeSubsystem.setIndexerVelocity(0), turbotakeSubsystem));
        leftBumper.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setShooterVelocity(1), () -> turbotakeSubsystem.setIndexerVelocity(0), turbotakeSubsystem));

    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
