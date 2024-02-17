// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ArmSubsystem;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    
    private Field2d field2d = new Field2d();

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
        Trigger leftJoystickDown = driverController.leftStick();
        Trigger aButton = driverController.a();
        Trigger bButton = driverController.b();
        Trigger xButton = driverController.x();
        Trigger yButton = driverController.y();

        Trigger rightJoystickDown = driverController.rightStick();

        // Elevator/joint position commands
        leftJoystickDown.onTrue(armSubsystem.jointStowPositionCommand());
        aButton.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0.1)));
        bButton.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(-0.1)));
        aButton.onFalse(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0)));
        bButton.onFalse(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0)));
        xButton.onTrue(armSubsystem.elevatorAmpPositionCommand());
        yButton.onTrue(armSubsystem.elevatorSpeakerPositionCommand());

        rightJoystickDown.whileTrue(armSubsystem.elevatorStowPositionCommand());

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
