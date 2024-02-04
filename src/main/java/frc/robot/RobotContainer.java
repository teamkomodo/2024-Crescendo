// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Subsystems
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

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

        Trigger bButton = driverController.b();
        bButton.whileTrue(Commands.run(() -> drivetrainSubsystem.drive(1.5, 0, 0, true, true), drivetrainSubsystem));
        Trigger xButton = driverController.x();
        xButton.whileTrue(Commands.run(() -> drivetrainSubsystem.drive(-1.5, 0, 0, true, true), drivetrainSubsystem));


        Trigger aButton = driverController.a();
        aButton.whileTrue(Commands.run(() -> drivetrainSubsystem.drive(2.5, 0, 0, true, true), drivetrainSubsystem));
        Trigger yButton = driverController.y();
        yButton.whileTrue(Commands.run(() -> drivetrainSubsystem.drive(-2.5, 0, 0, true, true), drivetrainSubsystem));

        // Trigger yButton = driverController.y();
        // yButton.whileTrue(Commands.run(() -> drivetrainSubsystem.runDriveVolts(12), drivetrainSubsystem));

    }
    
    public Command getAutonomousCommand() {
        
        PathPlannerPath path = PathPlannerPath.fromPathFile("Test");


        return AutoBuilder.followPath(path);

        //return autoChooser.getSelected();
    }
}
