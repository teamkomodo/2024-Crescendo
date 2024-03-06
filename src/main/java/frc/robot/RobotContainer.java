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
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TurbotakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

public class RobotContainer {    

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_XBOX_PORT);

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final TurbotakeSubsystem turbotakeSubsystem = new TurbotakeSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();    

    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

    // Driver Controls

        Trigger driverRT = driverController.rightTrigger();
        driverRT.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        driverRT.onFalse(drivetrainSubsystem.disableSlowModeCommand());

        Trigger driverStart = driverController.start();
        driverStart.onTrue(drivetrainSubsystem.zeroGyroCommand());

        driverController.b().onTrue(Commands.runOnce(() -> armSubsystem.setTurbotakeAngle(Rotation2d.fromDegrees(40))));

        // // deadband and curves are applied in command
        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> ( -driverController.getLeftY() ), // -Y on left joystick is +X for robot
                () -> ( -driverController.getLeftX() ), // -X on left joystick is +Y for robot
                () -> ( -driverController.getRightX() ) // -X on right joystick is +Z for robot
            )
        );

    // Operator Controls
        
        double shooterVelocity = 2000;
        double climberVelocity = 50;

        Trigger operatorA = operatorController.a();
        operatorA.onTrue(new IntakePositionCommand(armSubsystem));

        Trigger operatorB = operatorController.b();
        operatorB.onTrue(new StowPositionCommand(armSubsystem));

        Trigger operatorX = operatorController.x();
        operatorX.onTrue(new SpeakerPositionCommand(armSubsystem, drivetrainSubsystem));

        Trigger operatorY = operatorController.y();
        operatorY.onTrue(new AmpPositionCommand(armSubsystem));

        //amp
        Trigger operatorRB = operatorController.rightBumper();
        operatorRB.whileTrue(turbotakeSubsystem.shootForAmp());

        //intake indexer
        Trigger operatorLB = operatorController.leftBumper();
        operatorLB.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setIndexerPercent(1.0), () -> turbotakeSubsystem.setIndexerPercent(0)));
        
        //speaker
        Trigger operatorRT = operatorController.rightTrigger();
        operatorRT.whileTrue(turbotakeSubsystem.shootForSpeaker());

        //intake shooter
        Trigger operatorLT = operatorController.leftTrigger();
        operatorLT.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setShooterPercent(-0.5), () -> turbotakeSubsystem.setShooterPercent(0)));
        
        Trigger operatorRS = operatorController.rightStick();
        operatorRS.whileTrue(Commands.runEnd(() -> armSubsystem.setElevatorMotorPercent(operatorController.getRightX()), () -> armSubsystem.setElevatorPosition(armSubsystem.getElevatorPosition())));

        Trigger operatorLS = operatorController.leftStick();
        operatorLS.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(operatorController.getRightX()), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));
        
        // Trigger operatorStart = operatorController.start();
        // operatorStart.whileTrue(Commands.runEnd(() -> climberSubsystem.setMotorVelocity(climberVelocity), () -> climberSubsystem.holdMotorPosition()));

        // Trigger operatorBack = operatorController.back();
        // operatorBack.whileTrue(Commands.runEnd(() -> climberSubsystem.setMotorVelocity(-climberVelocity), () -> climberSubsystem.holdMotorPosition()));

    }
    
    public Command getAutonomousCommand() {
        return null;
    }

}
