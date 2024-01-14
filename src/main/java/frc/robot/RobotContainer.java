// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    
    private Field2d field2d = new Field2d();

    // Subsystems
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        // Triggers
        Trigger rTrigger = driverController.rightTrigger();

        //Slow/Fast Mode
        rTrigger.onTrue(drivetrainSubsystem.disableSlowModeCommand());
        rTrigger.onFalse(drivetrainSubsystem.enableSlowModeCommand());

        // Drivetrain controls
        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> (driverController.getLeftY()), // +Y on left joystick is +X for robot
                () -> (driverController.getLeftX()), // +X on left joystick is +Y for robot
                () -> (-driverController.getRightX())) // -X on right joystick is +Z for robot
        );
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
