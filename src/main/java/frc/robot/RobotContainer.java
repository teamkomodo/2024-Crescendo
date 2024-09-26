// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AimAssistCommand;
import frc.robot.commands.OffsetClimbCommand;
import frc.robot.commands.ProfiledClimbCommand;
import frc.robot.commands.positions.AmpPositionCommand;
import frc.robot.commands.positions.IntakePositionCommand;
import frc.robot.commands.positions.SpeakerPositionCommand;
import frc.robot.commands.positions.StowPositionCommand;
import frc.robot.commands.AimAssistCommand;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TurbotakeSubsystem;
import frc.robot.util.BlinkinPattern;
import frc.robot.subsystems.ClimberSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
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
    

    private final TeleopStateMachine teleopStateMachine = new TeleopStateMachine(drivetrainSubsystem, armSubsystem, turbotakeSubsystem, ledSubsystem, climberSubsystem, driverController.getHID(), operatorController.getHID());

    private final AimAssistCommand aimAssist = new AimAssistCommand();

    public RobotContainer() {
        configureBindings();
        registerNamedCommands();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }    
    
    private void configureBindings() {

        /*
         * Driver Controls
         * 
         * RT - Slow Mode
         * LB - Zero
         * 
         */

        Trigger driverRT = driverController.rightTrigger();

        driverRT.onTrue(drivetrainSubsystem.enableSlowModeCommand());
        driverRT.onFalse(drivetrainSubsystem.disableSlowModeCommand());

        Trigger driverLB = driverController.leftBumper();
        driverLB.onTrue(drivetrainSubsystem.zeroGyroCommand());

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
         * Operator Binds [ State Machine       | Manual Control ]
         * 
         *              A | Command Close Shoot | Stow           |
         *              B | Command Eject       | Pickup         |
         *              X | Command Aim Assist  | Amp            |
         *              Y | Command Far Shoot   | Close Speaker  |
         * 
         *   Left Stick X | EMPTY               | Elevator       |
         *  Right Stick Y | EMPTY               | Joint          |
         * 
         *  Right Trigger | Empty               | Shoot          |
         *   Left Trigger | Command Intake      | Intake         |
         * 
         *   Right Bumper | Command Spin Up     | EMPTY          |
         *    Left Bumper | Command Score Amp   | Outtake        |
         * 
         *          Start | Climber Up          | Climber Up     |
         *           Back | Climber Down        | Climber Down   |
         */

        Trigger operatorA = operatorController.a();
        operatorA.onTrue(dualBinding(
            teleopStateMachine.shootSpeakerCommand(),
            new StowPositionCommand(armSubsystem)
        ));

        
        
        Trigger operatorB = operatorController.b();
        operatorB.whileTrue(stateMachineBinding(
            teleopStateMachine.ejectCommand()
        ));

        operatorB.onTrue(manualBinding(
            new IntakePositionCommand(armSubsystem)
        ));

        Trigger operatorX = operatorController.x();
        // operatorX.whileTrue(stateMachineBinding(
        //     teleopStateMachine.alignAmpCommand()
        // ));
        
         operatorX.whileTrue(
            Commands.runEnd(() -> aimAssist.execute(), () -> aimAssist.end(true))
         );
        
        

        // operatorX.onTrue(manualBinding(
        //     new AmpPositionCommand(armSubsystem)
        // ));

        

        

        Trigger operatorY = operatorController.y();
        operatorY.onTrue(dualBinding(
            teleopStateMachine.shootSpeakerFarCommand(),
            new SpeakerPositionCommand(armSubsystem)
        ));

        Trigger operatorRB = operatorController.rightBumper();
        operatorRB.whileTrue(teleopStateMachine.spinUpCommand());
       //operatorRB.onTrue(shootCommand(20.786, 4000));

        Trigger operatorLB = operatorController.leftBumper();
        operatorLB.whileTrue(dualBinding(
            teleopStateMachine.scoreAmpCommand(),
            Commands.runEnd(() -> turbotakeSubsystem.setIndexerPercent(-1.0), () -> turbotakeSubsystem.setIndexerPercent(0))
        ));
        
        Trigger operatorRT = operatorController.rightTrigger();
        operatorRT.whileTrue(manualBinding(
            Commands.runEnd(() -> turbotakeSubsystem.setShooterVelocity(4500), () -> turbotakeSubsystem.setShooterPercent(0))
        ));

        Trigger operatorLT = operatorController.leftTrigger();
        operatorLT.whileTrue(dualBinding(
            teleopStateMachine.pickupGroundCommand(),
            Commands.runEnd(() -> turbotakeSubsystem.setIndexerPercent(1.0), () -> turbotakeSubsystem.setIndexerPercent(0))
        ));
        
        Trigger operatorLY = new Trigger(() -> Math.abs(operatorController.getLeftY()) > XBOX_DEADBAND);
        operatorLY.whileTrue(manualBinding(
            Commands.runEnd(() -> armSubsystem.setElevatorMotorPercent(-operatorController.getLeftY()), () -> armSubsystem.holdElevatorPosition())
        ));

        Trigger operatorRY = new Trigger(() -> Math.abs(operatorController.getRightY()) > XBOX_DEADBAND);
        operatorRY.whileTrue(manualBinding(
            Commands.runEnd(() -> armSubsystem.setJointMotorPercent(-operatorController.getRightY()), () -> armSubsystem.holdJointPosition())
        ));

        // Trigger operatorStart = operatorController.start();
        // operatorStart.whileTrue(
        //     Commands.parallel(
        //         new ProfiledClimbCommand(climberSubsystem, climberSubsystem.getExtendVelocity()),
        //         teleopStateMachine.extendClimbCommand()
        //     )
        // );

        // Trigger operatorBack = operatorController.back();
        // operatorBack.whileTrue(Commands.parallel(
        //         new ProfiledClimbCommand(climberSubsystem, climberSubsystem.getAscendVelocity()),
        //         teleopStateMachine.ascendClimbCommand()
        //     )
        // );

        armSubsystem.setJointMotorPercent(0);

        
        
    }

    public void teleopInit() {
        armSubsystem.setUseJointProfiledControl(false);
        armSubsystem.teleopInit();
        turbotakeSubsystem.teleopInit();
        climberSubsystem.setClimberDutyCycle(0);
        armSubsystem.setUseJointProfiledControl(false);
    }
    
    public Command getAutonomousCommand() {
        armSubsystem.setUseJointProfiledControl(true);
        if(autoChooser != null) {
           return autoChooser.getSelected();
        }

        return null;
    }

    public TeleopStateMachine getTeleopStateMachine() {
        return this.teleopStateMachine;
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand("armToIntake", Commands.sequence(
            Commands.print("intake pos"),
            armSubsystem.jointPositionCommand(16),
            armSubsystem.elevatorIntakePositionCommand(),
            Commands.waitSeconds(0.3),
            armSubsystem.jointIntakePositionCommand()
        ).deadlineWith(ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_VIOLET)));
        NamedCommands.registerCommand("armToStow", new StowPositionCommand(armSubsystem));
        NamedCommands.registerCommand("armToSpeaker", new SpeakerPositionCommand(armSubsystem));
        NamedCommands.registerCommand("shootSpeaker", shootCommand(JOINT_SPEAKER_POSITION, CLOSE_SHOOTER_SPEED));
        NamedCommands.registerCommand("runIndexerIn", Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0.4)));
        NamedCommands.registerCommand("stopIndexer", Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0)));
        NamedCommands.registerCommand("alignPiece", alignPieceCommand());
        NamedCommands.registerCommand("spinUp", Commands.sequence(
            Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(CLOSE_SHOOTER_SPEED)),
            Commands.waitUntil(() -> turbotakeSubsystem.checkShooterSpeed(CLOSE_SHOOTER_SPEED, 200))
        ).deadlineWith(ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.COLOR_1_PATTERN_LARSON_SCANNER)));
        NamedCommands.registerCommand("stopFlywheels", Commands.runOnce(() -> turbotakeSubsystem.setShooterPercent(0)));
        
        NamedCommands.registerCommand("shoot-C3", shootCommand(21, 4000));
        NamedCommands.registerCommand("shoot-C2", shootCommand(20, 4000));
        NamedCommands.registerCommand("shoot-C1", shootCommand(21.75, 4500));
        NamedCommands.registerCommand("ramp-C3", Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(4000)));
        NamedCommands.registerCommand("ramp-C2", Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(4000)));
        NamedCommands.registerCommand("ramp-C1", Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(4500)));

        NamedCommands.registerCommand("shoot-far-bottom", shootCommand(19, 3000));
        NamedCommands.registerCommand("ramp-far-bottom", Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(3000)));

        NamedCommands.registerCommand("shoot-close", shootCommand(JOINT_SPEAKER_POSITION, 2500));
        NamedCommands.registerCommand("stop-drivetrain", Commands.run(() -> drivetrainSubsystem.robotRelativeDrive(new ChassisSpeeds(0,0,0))));
        NamedCommands.registerCommand("smart-align-piece", smartAlignCommand());
    }

    private Command shootCommand(double jointPosition, double shooterSpeed){
        return Commands.sequence(
            Commands.print("joint: " + jointPosition + "shoot: " + shooterSpeed),
            armSubsystem.jointPositionCommand(jointPosition),
            armSubsystem.elevatorPositionCommand(0),
            Commands.runOnce(() -> turbotakeSubsystem.setShooterVelocity(shooterSpeed)),
            Commands.waitUntil(() -> armSubsystem.isJointAtPosition(jointPosition, 0.3) && turbotakeSubsystem.checkShooterSpeed(shooterSpeed, 200)),
            Commands.waitSeconds(0.4),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(1)),
            Commands.waitSeconds(0.25),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0))
        ).deadlineWith(ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.COLOR_1_PATTERN_LARSON_SCANNER));
    }

    private Command alignPieceCommand(){
        return Commands.sequence(
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0.1)),
            Commands.waitUntil(() -> turbotakeSubsystem.isPieceDetected()),
            Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(-0.2)),
            Commands.waitUntil(() -> !turbotakeSubsystem.isPieceDetected())
        ).withTimeout(1.0).finallyDo(() -> turbotakeSubsystem.setIndexerPercent(0));
    }

    private Command smartAlignCommand() {
        return Commands.waitUntil(() -> turbotakeSubsystem.getFilteredCurrent() > 15).finallyDo(() -> turbotakeSubsystem.setIndexerPercent(0));
    }

    private Command stateMachineBinding(Command stateMachineCommand) {
        return stateMachineCommand.unless(() -> !teleopStateMachine.isEnabled()).until(() -> !teleopStateMachine.isEnabled());
    }

    private Command manualBinding(Command manualBinding) {
        return manualBinding.unless(() -> teleopStateMachine.isEnabled()).until(() -> teleopStateMachine.isEnabled());
    }

    private Command dualBinding(Command stateMachineCommand, Command manualCommand) {
        return Commands.either(
            stateMachineCommand.until(() -> !teleopStateMachine.isEnabled()),
            manualCommand.until(teleopStateMachine::isEnabled),
            teleopStateMachine::isEnabled
        );
    }


    


    


}
