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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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
        //testing binds

        Trigger leftBumper = driverController.leftBumper();

        leftBumper.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        leftBumper.onFalse(drivetrainSubsystem.disableSlowModeCommand());

        Trigger startButton = driverController.start();
        startButton.onTrue(drivetrainSubsystem.zeroGyroCommand());

        // deadband and curves are applied in command

        Trigger aButton = driverController.a();
        aButton.whileTrue(teleopStateMachine.alignSpeakerCommand());

        Trigger bButton = driverController.b();
        bButton.whileTrue(teleopStateMachine.scoreAmpCommand());

        Trigger rightTrigger = driverController.rightTrigger();
        rightTrigger.whileTrue(teleopStateMachine.shootSpeakerCommand());

        Trigger rightBumper = driverController.rightBumper();
        rightBumper.whileTrue(teleopStateMachine.pickupGroundCommand());

        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> ( -driverController.getLeftY() ), // -Y on left joystick is +X for robot
                () -> ( -driverController.getLeftX() ), // -X on left joystick is +Y for robot
                () -> ( -driverController.getRightX() ) // -X on right joystick is +Z for robot
            )
        );
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
