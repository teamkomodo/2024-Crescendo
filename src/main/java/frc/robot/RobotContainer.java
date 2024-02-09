// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.TurboTakeSubsystem;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.TimedRobot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer extends TimedRobot {
    
    

    // Subsystems
    public final TurboTakeSubsystem turbotakesubsystem = new TurboTakeSubsystem();
    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        



        //Turbotake Binds
        Trigger rightBumper = driverController.rightBumper();
        Trigger leftBumper = driverController.leftBumper();

        //when true set motors at
        rightBumper.onTrue(Commands.runOnce(() -> {turbotakesubsystem.setIndexSpeed(INDEXER_SPEED);}));
        leftBumper.onTrue(Commands.runOnce(() -> {turbotakesubsystem.setShootSpeed(SHOOTER_SPEED);}));

        //when false disable right or left
        rightBumper.onFalse(Commands.runOnce(() -> {turbotakesubsystem.setIndexSpeed(0);}));
        leftBumper.onFalse(Commands.runOnce(() -> {turbotakesubsystem.setShootSpeed(0);}));

        Trigger aButton = driverController.a();
        Trigger bButton = driverController.b();

        aButton.onTrue( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(INDEXER_SPEED) ) );
        bButton.onTrue( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(-INDEXER_SPEED) ) );
        aButton.onFalse( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(0) ) );
        bButton.onFalse( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(0) ) );

    }

    @Override
    public void teleopPeriodic(){
        turbotakesubsystem.updateShuffleboard();
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
