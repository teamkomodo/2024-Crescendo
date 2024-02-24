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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.*;

public class RobotContainer {    

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT); 

    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    public final TurbotakeSubsystem turbotakeSubsystem = new TurbotakeSubsystem();
    private final LEDSubsystem ledSubsystem = new LEDSubsystem();
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();    

    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        //SysID testing binds
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

        //motor buttons
        Trigger rightBumper = driverController.rightBumper();//intake
        Trigger leftBumper = driverController.leftBumper();//speaker
        Trigger xbutton = driverController.x();//trap/amp button
       

        //shooter velocity
        leftBumper.onTrue(Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(2500)));
        leftBumper.onFalse(Commands.runOnce(() -> turbotakeSubsystem.turnoffShooter()));


        //indexer duty cycle
        rightBumper.onTrue(Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(1)));
        rightBumper.onFalse(Commands.runOnce(() -> turbotakeSubsystem.turnoffIndexer()));
        //trap/amp
        xbutton.onTrue(Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(-1)));
        xbutton.onFalse(Commands.runOnce(() -> turbotakeSubsystem.turnoffIndexer()));
        
    }
    
    public Command getAutonomousCommand() {
        return null;
    }

}
