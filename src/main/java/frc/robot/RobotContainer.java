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
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

public class RobotContainer {    

    private final SendableChooser<Command> autoChooser;

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 
    private final CommandXboxController operatorController = new CommandXboxController(OPERATOR_XBOX_PORT);

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final TurbotakeSubsystem turbotakeSubsystem = new TurbotakeSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();    

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
        configureBindings();
    }

    private void configureNamedCommands() {
        NamedCommands.registerCommand("speakerposition", new SpeakerPositionCommand(armSubsystem));
        NamedCommands.registerCommand("shoot", Commands.sequence(
            turbotakeSubsystem.spinUpFlywheelCommand(1500, 50),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(-0.5)),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0)),
            Commands.runOnce(() -> turbotakeSubsystem.setShooterPercent(0))
        ));
        NamedCommands.registerCommand("stowposition", new StowPositionCommand(armSubsystem));
    }
    
    private void configureBindings() {

        /*
         * Driver Controls
         * 
         * A - Align Speaker
         * B - Score Amp
         * 
         * RT - Shoot Speaker
         * 
         * RB - Pickup Ground
         * LB - Slow Mode
         */

        Trigger driverLB = driverController.leftBumper();

        driverLB.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        driverLB.onFalse(drivetrainSubsystem.disableSlowModeCommand());

        Trigger driverStart = driverController.start();
        driverStart.onTrue(drivetrainSubsystem.zeroGyroCommand());

        // deadband and curves are applied in command
        drivetrainSubsystem.setDefaultCommand(
            drivetrainSubsystem.joystickDriveCommand(
                () -> ( -driverController.getLeftY() ), // -Y on left joystick is +X for robot
                () -> ( -driverController.getLeftX() ), // -X on left joystick is +Y for robot
                () -> ( -driverController.getRightX() ) // -X on right joystick is +Z for robot
            )
        );

        /*
         * Operator Controls
         * 
         * A - Stow
         * B - Intake
         * X - Amp
         * Y - Speaker
         * 
         * Left Stick X - Elevator
         * Right Stick Y - Joint
         * 
         * Right Trigger - Shoot
         * Left Trigger - Indexer
         * 
         * Right Bumper - Reverse Shooter
         * Left Bumper - Reverse Indexer
         */
        
        double climberVelocity = 50;

        Trigger operatorA = operatorController.a();
        operatorA.onTrue(new StowPositionCommand(armSubsystem));
        //operatorA.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(0.8), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));

        Trigger operatorB = operatorController.b();
        operatorB.onTrue(new IntakePositionCommand(armSubsystem));
        //operatorB.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(-0.2), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));

        Trigger operatorX = operatorController.x();
        operatorX.onTrue(new AmpPositionCommand(armSubsystem));

        Trigger operatorY = operatorController.y();
        operatorY.onTrue(new SpeakerPositionCommand(armSubsystem));

        Trigger operatorRB = operatorController.rightBumper();
        operatorRB.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setShooterPercent(-1), () -> turbotakeSubsystem.setShooterPercent(0)));

        Trigger operatorLB = operatorController.leftBumper();
        operatorLB.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setIndexerPercent(-0.5), () -> turbotakeSubsystem.setIndexerPercent(0)));
        
        Trigger operatorRT = operatorController.rightTrigger();
        operatorRT.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setShooterPercent(1, 0.5), () -> turbotakeSubsystem.setShooterPercent(0)));

        Trigger operatorLT = operatorController.leftTrigger();
        operatorLT.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setIndexerPercent(1.0), () -> turbotakeSubsystem.setIndexerPercent(0)));
        
        Trigger operatorRSY = operatorController.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1);
        operatorRSY.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(operatorController.getRightY()), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));

        Trigger operatorLSX = operatorController.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1);
        operatorLSX.whileTrue(Commands.runEnd(() -> armSubsystem.setElevatorMotorPercent(operatorController.getLeftX()), () -> armSubsystem.setElevatorPosition(armSubsystem.getElevatorPosition())));
        
        Trigger operatorStart = operatorController.start();
        operatorStart.whileTrue(Commands.runEnd(() -> climberSubsystem.setMotorVelocity(climberVelocity), () -> climberSubsystem.holdMotorPosition()));

        Trigger operatorBack = operatorController.back();
        operatorBack.whileTrue(Commands.runEnd(() -> climberSubsystem.setMotorVelocity(-climberVelocity), () -> climberSubsystem.holdMotorPosition()));

    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
