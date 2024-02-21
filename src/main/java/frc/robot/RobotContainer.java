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

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Subsystems

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
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
        Trigger rightTrigger = driverController.rightTrigger();
        Trigger leftTrigger = driverController.leftTrigger();
        Trigger rightBumper = driverController.rightBumper();
        Trigger leftBumper = driverController.leftBumper();


        // Elevator/joint position commands
        aButton.onTrue(armSubsystem.jointAmpPositionCommand());
        bButton.onTrue(armSubsystem.jointIntakePositionCommand());
        xButton.onTrue(armSubsystem.elevatorAmpPositionCommand());
        yButton.onTrue(armSubsystem.elevatorSpeakerPositionCommand());
        rightTrigger.onTrue(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(0.5)));
        rightTrigger.onFalse(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(0)));
        leftTrigger.onTrue(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(-0.5)));
        leftTrigger.onFalse(Commands.runOnce(() -> armSubsystem.setJointMotorPercent(0)));
        rightBumper.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0.5)));
        rightBumper.onFalse(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0)));
        leftBumper.onTrue(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(-0.5)));
        leftBumper.onFalse(Commands.runOnce(() -> armSubsystem.setElevatorMotorPercent(0)));

        

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
        return autoChooser.getSelected();
    }
}
