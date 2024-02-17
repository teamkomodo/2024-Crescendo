// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.ClimberSubsystem;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

    // Subsystems
    private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

    //Inputs Devices
    private final CommandXboxController driverController = new CommandXboxController(DRIVER_XBOX_PORT);    

    public RobotContainer() {
        configureBindings();
    }
    
    private void configureBindings() {

        Trigger leftTrigger = driverController.leftTrigger();
        Trigger rightTrigger = driverController.rightTrigger();
        Trigger xButton = driverController.x();
        Trigger bButton = driverController.b();
        
        rightTrigger.onTrue(climberSubsystem.climbPositionCommand());
        leftTrigger.onTrue(climberSubsystem.climb());
        xButton.onTrue(Commands.runOnce(() -> climberSubsystem.setMotorDutyCycle(-0.3)));
        xButton.onFalse(Commands.runOnce(() -> climberSubsystem.setMotorDutyCycle(0)));
        bButton.onTrue(Commands.runOnce(() -> climberSubsystem.setMotorDutyCycle(0.3)));
        bButton.onFalse(Commands.runOnce(() -> climberSubsystem.setMotorDutyCycle(0)));

    }

    public Command getAutonomousCommand() {
        return null;
    }
    
}
