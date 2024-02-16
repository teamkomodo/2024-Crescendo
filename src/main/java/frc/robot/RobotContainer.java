// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.States.ShootAmpState;
import frc.robot.commands.States.ShootSpeakerState;
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
    
    

    // Subsystem
    public final TurboTakeSubsystem turbotakesubsystem = new TurboTakeSubsystem();

    //States
    public final ShootAmpState shootampState = new ShootAmpState(turbotakesubsystem);
    public final ShootSpeakerState shootspeakerState = new ShootSpeakerState(turbotakesubsystem);

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

        //run state's sequence
        aButton.onTrue(shootampState);
        bButton.onTrue(shootspeakerState);
        

        //runs the motors directly
        rightBumper.whileTrue(Commands.runOnce(() -> {turbotakesubsystem.setIndexerVelocity(INDEXER_SPEED);}));
        leftBumper.whileTrue(Commands.runOnce(() -> {turbotakesubsystem.setShooterVelocity(SHOOTER_SPEED);}));

        //tests shooters
        rightTrigger.whileTrue(Commands.sequence(
                turbotakesubsystem.shooterRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5), 
                turbotakesubsystem.shooterRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                new WaitCommand(5),
                turbotakesubsystem.shooterRoutine.dynamic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5),
                turbotakesubsystem.shooterRoutine.dynamic(SysIdRoutine.Direction.kReverse)
                ));


        //tests indexer
        leftTrigger.whileTrue(Commands.sequence(
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
        
    }
    
    public Command getAutonomousCommand() {
        return null;
    }
}
