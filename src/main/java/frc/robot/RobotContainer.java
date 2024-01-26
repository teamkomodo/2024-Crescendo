// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.positions.AmpPositionCommand;
import frc.robot.commands.positions.IntakePositionCommand;
import frc.robot.commands.positions.SpeakerPositionCommand;
import frc.robot.commands.positions.StowPositionCommand;
import frc.robot.commands.positions.TrapPositionCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.JointSubsystem;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    
    private Field2d field2d = new Field2d();

    // Subsystems
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    private final JointSubsystem jointSubsystem = new JointSubsystem();

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        // Triggers
        Trigger rTrigger = driverController.rightTrigger();
        Trigger leftJoystickDown = driverController.leftStick();
        Trigger aButton = driverController.a();
        Trigger bButton = driverController.b();
        Trigger xButton = driverController.x();
        Trigger yButton = driverController.y();

        leftJoystickDown.onTrue(new StowPositionCommand(jointSubsystem));
        aButton.onTrue(new TrapPositionCommand(jointSubsystem));
        bButton.onTrue(new IntakePositionCommand(jointSubsystem));
        xButton.onTrue(new AmpPositionCommand(jointSubsystem));
        yButton.onTrue(new SpeakerPositionCommand(jointSubsystem));

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
