// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.TurboTakeSubsystem;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.TimedRobot;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer extends TimedRobot {
    
    

    // Subsystems
    public final TurboTakeSubsystem turbotakesubsystem = new TurboTakeSubsystem();
    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    
    
    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        


        //competition binds
        //Turbotake Binds
        // Trigger rightBumper = driverController.rightBumper();
        // Trigger leftBumper = driverController.leftBumper();

        // //when true set motors at
        // rightBumper.onTrue(Commands.runOnce(() -> {turbotakesubsystem.setIndexSpeed(INDEXER_SPEED);}));
        // leftBumper.onTrue(Commands.runOnce(() -> {turbotakesubsystem.setShootSpeed(SHOOTER_SPEED);}));

        // //when false disable right or left
        // rightBumper.onFalse(Commands.runOnce(() -> {turbotakesubsystem.setIndexSpeed(0);}));
        // leftBumper.onFalse(Commands.runOnce(() -> {turbotakesubsystem.setShootSpeed(0);}));


        //Current testing binds

        //indexer button
        Trigger aButton = driverController.a();
        Trigger bButton = driverController.b();

        aButton.onTrue( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(INDEXER_SPEED) ) );
        bButton.onTrue( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(-INDEXER_SPEED) ) );
        aButton.onFalse( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(0) ) );
        bButton.onFalse( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(0) ) );

        Trigger rightTrigger = driverController.rightTrigger();
        rightTrigger.whileTrue( Commands.run( () -> turbotakesubsystem.setIndexSpeed(driverController.getRightTriggerAxis() * INDEXER_SPEED), turbotakesubsystem ) );
        rightTrigger.onFalse( Commands.runOnce( () -> turbotakesubsystem.setIndexSpeed(0) ) );

        //shooter button
        Trigger xButton = driverController.x();
        Trigger yButton = driverController.y();

        xButton.onTrue( Commands.runOnce( () -> turbotakesubsystem.SetShooterSpeed(SHOOTER_SPEED) ) );
        yButton.onTrue(Commands.runOnce( () -> turbotakesubsystem.SetShooterSpeed(-SHOOTER_SPEED) ) );
        xButton.onFalse( Commands.runOnce( () -> turbotakesubsystem.SetShooterSpeed(0) ) );
        yButton.onFalse( Commands.runOnce( () -> turbotakesubsystem.SetShooterSpeed(0) ) );
        
        Trigger rightBumper = driverController.rightBumper();
        Trigger leftBumper = driverController.leftBumper();

        
        rightBumper.whileTrue(Commands.sequence(
                turbotakesubsystem.shooterRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5), 
                turbotakesubsystem.shooterRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                new WaitCommand(5),
                turbotakesubsystem.shooterRoutine.dynamic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5),
                turbotakesubsystem.shooterRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                ));


        leftBumper.whileTrue(Commands.sequence(
                turbotakesubsystem.indexerRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5), 
                turbotakesubsystem.indexerRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                new WaitCommand(5),
                turbotakesubsystem.indexerRoutine.dynamic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5),
                turbotakesubsystem.indexerRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                ));

        



    }

    @Override
    public void teleopPeriodic(){
        turbotakesubsystem.updateShuffleboard();
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
