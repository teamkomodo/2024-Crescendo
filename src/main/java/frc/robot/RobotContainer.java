// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.positions.IntakePositionCommand;
import frc.robot.commands.positions.SpeakerPositionCommand;
import frc.robot.commands.positions.StowPositionCommand;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TurbotakeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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

    private final TeleopStateMachine teleopStateMachine = new TeleopStateMachine(drivetrainSubsystem, armSubsystem, turbotakeSubsystem, ledSubsystem, driverController.getHID(), operatorController.getHID());

    public RobotContainer() {
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();


        SmartDashboard.putData("Auto Chooser",autoChooser);
    }

    
      private final SendableChooser<Command> autoChooser;
    
    
    private void configureBindings() {
    //Auto
        NamedCommands.registerCommand("armToIntake", Commands.sequence(
            armSubsystem.jointStowPositionCommand(),
            Commands.waitSeconds(0.2),
            armSubsystem.elevatorIntakePositionCommand(),
            Commands.waitSeconds(0.2),
            armSubsystem.jointIntakePositionCommand()
        ));
        NamedCommands.registerCommand("armToStow", new StowPositionCommand(armSubsystem));
        NamedCommands.registerCommand("armToSpeaker", new SpeakerPositionCommand(armSubsystem));
        NamedCommands.registerCommand("intakePiece", intakeCommand());
        NamedCommands.registerCommand("shootSpeaker", shootCommand());
        NamedCommands.registerCommand("runIndexerIn", Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0.5)));
        NamedCommands.registerCommand("runIndexerStop", Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0)));
        NamedCommands.registerCommand("alignPiece", alignPieceCommand());


        /*
         * Driver Controls
         * 
         * RT - Slow Mode
         * 
         */

        Trigger driverRT = driverController.rightTrigger();

        driverRT.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        driverRT.onFalse(drivetrainSubsystem.disableSlowModeCommand());

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

        driverController.a().whileTrue(drivetrainSubsystem.pointToSpeakerCommand());

        /*
         * Operator Controls
         * 
         * A - Stow
         * B - Command Eject
         * X - Command Align Amp
         * 
         * Left Stick X - Elevator
         * Right Stick Y - Joint
         * 
         * Right Trigger - Command Shoot State
         * Left Trigger - Command Intake State
         * 
         * Right Bumper - Command Spin Up State
         * Left Bumper - Command Score Amp
         */

        Trigger operatorA = operatorController.a();
        operatorA.onTrue(new StowPositionCommand(armSubsystem));
        //operatorA.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(0.8), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));

        Trigger operatorB = operatorController.b();
        operatorB.whileTrue(teleopStateMachine.ejectCommand());

        Trigger operatorX = operatorController.x();
        operatorX.whileTrue(teleopStateMachine.alignAmpCommand());

        Trigger operatorRB = operatorController.rightBumper();
        operatorRB.whileTrue(teleopStateMachine.spinUpCommand());

        Trigger operatorLB = operatorController.leftBumper();
        operatorLB.whileTrue(teleopStateMachine.scoreAmpCommand());
        
        Trigger operatorRT = operatorController.rightTrigger();
        operatorRT.whileTrue(teleopStateMachine.shootSpeakerCommand());

        Trigger operatorLT = operatorController.leftTrigger();
        operatorLT.whileTrue(teleopStateMachine.pickupGroundCommand());
        
        Trigger operatorRSY = new Trigger(() -> (Math.abs(operatorController.getRightY()) > XBOX_DEADBAND));
        operatorRSY.whileTrue(Commands.runEnd(() -> armSubsystem.setJointMotorPercent(-operatorController.getRightY()), () -> armSubsystem.setJointPosition(armSubsystem.getJointPosition())));

        Trigger operatorLSX = new Trigger(() -> (Math.abs(operatorController.getLeftX()) > XBOX_DEADBAND));
        operatorLSX.whileTrue(Commands.runEnd(() -> armSubsystem.setElevatorMotorPercent(operatorController.getLeftX()), () -> armSubsystem.setElevatorPosition(armSubsystem.getElevatorPosition())));
        
        Trigger operatorStart = operatorController.start();
        operatorStart.whileTrue(Commands.runEnd(() -> climberSubsystem.setClimberDutyCycle(0.3), () -> climberSubsystem.holdClimberPosition()));

        Trigger operatorBack = operatorController.back();
        operatorBack.whileTrue(Commands.runEnd(() -> climberSubsystem.setClimberDutyCycle(-0.3), () -> climberSubsystem.holdClimberPosition()));

        armSubsystem.setJointMotorPercent(0);
    }

    public void teleopInit() {
        armSubsystem.teleopInit();
        turbotakeSubsystem.teleopInit();
    }
    
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public TeleopStateMachine getTeleopStateMachine() {
        return this.teleopStateMachine;
    }

    public Command shootCommand(){
        return Commands.sequence(
            new SpeakerPositionCommand(armSubsystem),
            Commands.runOnce(() -> System.out.println("shooting")),
            Commands.sequence(
                Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(SPEAKER_SPEED)),
                Commands.waitSeconds(1.75),
                Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(1)),
                Commands.waitSeconds(1)
            ).finallyDo(() -> {
                turbotakeSubsystem.setIndexerPercent(0);
                turbotakeSubsystem.turnoffShooter();
                System.out.println("canceled");
            })
        );
    }

    public Command intakeCommand() {
        return Commands.sequence(
            new IntakePositionCommand(armSubsystem),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(1.0)),
            Commands.waitSeconds(1.0),
            Commands.waitUntil(() -> turbotakeSubsystem.getFilteredCurrent() > 15),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0.2)),
            new SpeakerPositionCommand(armSubsystem),
            Commands.waitUntil(() -> turbotakeSubsystem.isPieceDetected()),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(-0.1)),
            Commands.waitUntil(() -> !turbotakeSubsystem.isPieceDetected()),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0))
        );
    }

    public Command alignPieceCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0.2)),
            Commands.waitUntil(() -> turbotakeSubsystem.isPieceDetected()),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(-0.2)),
            Commands.waitUntil(() -> !turbotakeSubsystem.isPieceDetected())
        ).withTimeout(1.0).finallyDo(() -> turbotakeSubsystem.setIndexerPercent(0));
    }

}
