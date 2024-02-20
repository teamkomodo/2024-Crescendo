// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;

import static frc.robot.Constants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Subsystems
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

    private final TeleopStateMachine teleopStateMachine = new TeleopStateMachine(drivetrainSubsystem);

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

        Trigger leftBumper = driverController.leftBumper();

        leftBumper.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        leftBumper.onFalse(drivetrainSubsystem.disableSlowModeCommand());

        Trigger startButton = driverController.start();
        startButton.onTrue(drivetrainSubsystem.zeroGyroCommand());

        // deadband and curves are applied in command
        // drivetrainSubsystem.setDefaultCommand(
        //     drivetrainSubsystem.joystickDriveCommand(
        //         () -> ( -driverController.getLeftY() ), // -Y on left joystick is +X for robot
        //         () -> ( -driverController.getLeftX() ), // -X on left joystick is +Y for robot
        //         () -> ( -driverController.getRightX() ) // -X on right joystick is +Z for robot
        //     )
        // );

        Trigger aButton = driverController.a();
        aButton.whileTrue(teleopStateMachine.alignSpeakerCommand());

        Trigger bButton = driverController.b();
        bButton.whileTrue(teleopStateMachine.scoreAmpCommand());

        Trigger rightTrigger = driverController.rightTrigger();
        rightTrigger.whileTrue(teleopStateMachine.shootSpeakerCommand());

        Trigger rightBumper = driverController.rightBumper();
        rightBumper.whileTrue(teleopStateMachine.pickupGroundCommand());

        Trigger xButton = driverController.b();
        CANSparkMax intake = new CANSparkMax(35, MotorType.kBrushless);
        xButton.whileTrue(Commands.runEnd(() -> intake.set(-1), () -> intake.set(0)));

    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
