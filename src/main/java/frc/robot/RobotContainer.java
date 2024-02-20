// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.subsystems.TurbotakeSubsystem;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.TimedRobot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer extends TimedRobot {
    
    // Subsystem
    public final TurbotakeSubsystem turbotakeSubsystem = new TurbotakeSubsystem();

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {
        //testing binds

        //state buttons
        Trigger aButton = driverController.a();//amp
        Trigger bButton = driverController.b();//speaker

        //SysID testing binds
        Trigger rightTrigger = driverController.rightTrigger();
        Trigger leftTrigger = driverController.leftStick();


        //motor buttons
        Trigger rightBumper = driverController.rightBumper();
        Trigger leftBumper = driverController.leftBumper();

        //run duty cycles
        aButton.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setIndexerPercent(1), () -> turbotakeSubsystem.setIndexerPercent(0), turbotakeSubsystem));
        bButton.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setShooterPercent(1), () -> turbotakeSubsystem.setShooterPercent(0), turbotakeSubsystem));
        

        //runs the motors directly
        rightBumper.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setIndexerVelocity(1), () -> turbotakeSubsystem.setIndexerVelocity(0), turbotakeSubsystem));
        leftBumper.whileTrue(Commands.runEnd(() -> turbotakeSubsystem.setShooterVelocity(1), () -> turbotakeSubsystem.setIndexerVelocity(0), turbotakeSubsystem));

        //tests shooters
        rightTrigger.whileTrue(Commands.runOnce(() -> turbotakeSubsystem.shooterSysIdCommand()));

        //tests indexer
        leftTrigger.whileTrue(Commands.runOnce(() -> turbotakeSubsystem.indexerSysIdCommand()));
    }

    @Override
    public void teleopPeriodic(){
        
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
