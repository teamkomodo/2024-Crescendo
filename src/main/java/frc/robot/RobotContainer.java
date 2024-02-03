// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurboTakeSubsystem;
import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    
    private Field2d field2d = new Field2d();

    // Subsystems
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(field2d);
    public final TurboTakeSubsystem turbotakesubsystem = new TurboTakeSubsystem();
    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        Trigger rightTrigger = driverController.rightTrigger();

        rightTrigger.onTrue(drivetrainSubsystem.disableSlowModeCommand());
        rightTrigger.onFalse(drivetrainSubsystem.enableSlowModeCommand());

        Trigger startButton = driverController.start();
        startButton.onTrue(drivetrainSubsystem.zeroGyroCommand());

        double deadband = 0.1;

        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> ( MathUtil.applyDeadband(-driverController.getLeftY(), deadband) ), // -Y on left joystick is +X for robot
                () -> ( MathUtil.applyDeadband(-driverController.getLeftX(), deadband) ), // -X on left joystick is +Y for robot
                () -> ( MathUtil.applyDeadband(-driverController.getRightX(), deadband) )) // -X on right joystick is +Z for robot
        );

        //Turbotake Binds
        Trigger rightBumper = driverController.rightBumper();
        Trigger leftBumper = driverController.leftBumper();

        //when true set motors at
        rightBumper.onTrue(Commands.runOnce(() -> {turbotakesubsystem.SetIndexerSpeed(INDEXER_SPEED);}));
        leftBumper.onTrue(Commands.runOnce(() -> {turbotakesubsystem.SetShooterSpeed(SHOOTER_SPEED);}));

        //when false disable right or left
        rightBumper.onFalse(Commands.runOnce(() -> {turbotakesubsystem.SetIndexerSpeed(0);}));
        leftBumper.onFalse(Commands.runOnce(() -> {turbotakesubsystem.SetShooterSpeed(0);}));



    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
