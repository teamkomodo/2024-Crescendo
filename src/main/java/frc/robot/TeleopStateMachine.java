package frc.robot;

import static frc.robot.Constants.ON_RED_ALLIANCE;
import static frc.robot.Constants.SHOOTER_MAX_VELOCITY;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.positions.AmpPositionCommand;
import frc.robot.commands.positions.IntakePositionCommand;
import frc.robot.commands.positions.SpeakerPositionCommand;
import frc.robot.commands.positions.StowPositionCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TurbotakeSubsystem;
import frc.robot.util.BlinkinPattern;

public class TeleopStateMachine {
    
    // Enum to represent each of the states in our state machine
    public static enum State {
        START,
        DRIVE_WITHOUT_PIECE,
        DRIVE_WITH_PIECE,
        PICKUP_GROUND,
        SCORE_SPEAKER,
        SCORE_AMP
    }

    public static enum ShootingState {
        PREPARE_SHOOT,
        READY_SHOOT,
        ALIGN_SPEAKER,
        SHOOT_SPEAKER,
    }

    public static enum PickupState {
        NO_PIECE,
        PARTIAL_AQUISITION,
        ALIGN_PIECE
    }

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("teleopstatemachine");
    private final StringPublisher statePublisher = table.getStringTopic("teleopstate").publish();
    private final StringPublisher shootingStatePublisher = table.getStringTopic("shootingstate").publish();
    private final StringPublisher pickupStatePublisher = table.getStringTopic("pickupstate").publish();

    // Store a reference to the Command Scheduler so it's easier to schedule commands
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private final Timer timer = new Timer();

    private final Command scoreAmpCommand;
    
    // Store references to each of the subsystems, so that we can schedule their commands.
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ArmSubsystem armSubsystem;
    private final TurbotakeSubsystem turbotakeSubsystem;
    private final LEDSubsystem ledSubsystem;

    private final XboxController driverController;

    // Store the current state
    private State currentState = State.START;
    private ShootingState currentShootingState = ShootingState.PREPARE_SHOOT;
    private PickupState currentPickupState = PickupState.NO_PIECE;

    // This will turn to true when the current state's exit condition is met, and will signal the next state to run its entrance code
    private boolean stateSwitched = true;
    private boolean shootingStateSwitched = true;
    private boolean currentPickupStateSwitched = true;

    private boolean commandingPickupGround = false;
    private boolean commandingAlignSpeaker = false;
    private boolean commandingShootSpeaker = false;
    private boolean commandingScoreAmp = false;

    public TeleopStateMachine(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, TurbotakeSubsystem turbotakeSubsystem, LEDSubsystem ledSubsystem, XboxController driverController) {

        // Set our references
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.turbotakeSubsystem = turbotakeSubsystem;
        this.ledSubsystem = ledSubsystem;

        this.driverController = driverController;

        scoreAmpCommand = Commands.sequence(
            AutoBuilder.pathfindToPoseFlipped(new Pose2d(0, 0, Rotation2d.fromDegrees(90)), null),
            new AmpPositionCommand(armSubsystem),
            Commands.waitSeconds(0.1),
            Commands.runOnce( () -> turbotakeSubsystem.setIndexerPercent(-1) )
        );

    }

    public void init() {
        currentState = State.START;
        stateSwitched = true;

        currentShootingState = ShootingState.PREPARE_SHOOT;
        shootingStateSwitched = true;

        currentPickupStateSwitched = true;
        currentPickupState = PickupState.NO_PIECE;

        commandingPickupGround = false;
        commandingAlignSpeaker = false;
        commandingShootSpeaker = false;
        commandingScoreAmp = false;
    }

    public void periodic() {

        updateTelemetry();

        // Switch through all of all states to determine what code to execute this iteration
        switch (currentState) {
            case START:
                currentState = turbotakeSubsystem.isPieceDetected()? State.DRIVE_WITH_PIECE : State.DRIVE_WITHOUT_PIECE;
                stateSwitched = true;
                // intentionally no break statement
            case DRIVE_WITHOUT_PIECE:

                // Did we just enter this state?
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(ledSubsystem.setPatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_1_ON_COLOR_2));
                }


                // Check exit conditions
                if(commandingPickupGround) {
                    currentState = State.PICKUP_GROUND;
                    stateSwitched = true;
                }

                break;
            case DRIVE_WITH_PIECE:
                
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(ledSubsystem.setPatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1));
                }

                if( (drivetrainSubsystem.getPose().getX() < 8.5 && !ON_RED_ALLIANCE.getAsBoolean()) || (drivetrainSubsystem.getPose().getX() > 8.5 && ON_RED_ALLIANCE.getAsBoolean())) {
                    stateSwitched = true;
                    currentState = State.SCORE_SPEAKER;
                }

                // if(!turbotakeSubsystem.hasPiece() && turbotakeSubsystem.getIndexerVelocity() < -1000) {
                //     stateSwitched = true;
                //     currentState = State.DRIVE_WITHOUT_PIECE;
                // }

                if(commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.SCORE_AMP;
                }

                break;
            case PICKUP_GROUND:

                if(stateSwitched) {
                    stateSwitched = false;
                    currentPickupState = PickupState.NO_PIECE;
                    currentPickupStateSwitched = true;
                    commandScheduler.schedule(ledSubsystem.setPatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_COLOR_WAVES_FOREST_PALETTE));
                }

                // Internal State Machine

                final double indexerRampUpTime = 2.0;
                final double intakeCurrentThreshold = 15;

                switch (currentPickupState) {
                    case NO_PIECE:

                        if(currentPickupStateSwitched) {
                            currentPickupStateSwitched = false;
                            timer.reset();
                            timer.start();
                            commandScheduler.schedule(
                                new IntakePositionCommand(armSubsystem),
                                Commands.sequence(
                                    Commands.waitUntil(() -> (armSubsystem.isJointZeroed() && armSubsystem.isElevatorZeroed())),
                                    Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0.8))
                                )                                
                            );
                            
                        }

                        if(!commandingPickupGround) {
                            stateSwitched = true;
                            currentState = State.DRIVE_WITHOUT_PIECE;
                        }

                        if(timer.hasElapsed(indexerRampUpTime) && turbotakeSubsystem.getFilteredCurrent() - intakeCurrentThreshold > 0) {
                            currentPickupStateSwitched = true;
                            currentPickupState = PickupState.PARTIAL_AQUISITION;
                        }

                        break;
                    case PARTIAL_AQUISITION:
                        
                        if(currentPickupStateSwitched) {
                            currentPickupStateSwitched = false;
                            commandScheduler.schedule(new StowPositionCommand(armSubsystem));
                            turbotakeSubsystem.setIndexerPercent(0.2);
                        }

                        if(turbotakeSubsystem.isPieceDetected()) {
                            currentPickupStateSwitched = true;
                            currentPickupState = PickupState.ALIGN_PIECE;
                            timer.stop();
                        }

                        break;
                    case ALIGN_PIECE:
                        
                        if(currentPickupStateSwitched) {
                            currentPickupStateSwitched = false;
                            turbotakeSubsystem.setIndexerPercent(-0.05);
                        }

                        if(!turbotakeSubsystem.isPieceDetected()) {
                            stateSwitched = true;
                            currentState = State.DRIVE_WITH_PIECE;
                        }

                        break;
                }

                if(stateSwitched) {
                    commandScheduler.schedule(new StowPositionCommand(armSubsystem));
                    turbotakeSubsystem.setIndexerPercent(0);
                }
                
                break;
            case SCORE_SPEAKER:

                if(stateSwitched)  {
                    stateSwitched = false;
                    shootingStateSwitched = true;
                    currentShootingState = ShootingState.PREPARE_SHOOT;
                }

                // Interal State Machine
                switch (currentShootingState) {
                    case PREPARE_SHOOT:

                        double shooterThreshold = SHOOTER_MAX_VELOCITY - 50;

                        if(shootingStateSwitched) {
                            shootingStateSwitched = false;
                            turbotakeSubsystem.setShooterVelocity(SHOOTER_MAX_VELOCITY);
                            commandScheduler.schedule(ledSubsystem.setPatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_BREATH_BLUE));
                        }

                        if(turbotakeSubsystem.getShooterVelocity() > shooterThreshold) {
                            shootingStateSwitched = true;
                            currentShootingState = ShootingState.READY_SHOOT;
                        }

                        break;
                    case READY_SHOOT:
                        
                        if(shootingStateSwitched) {
                            shootingStateSwitched = false;
                            commandScheduler.schedule(
                                ledSubsystem.setPatternCommand(BlinkinPattern.SOLID_COLORS_GREEN),
                                Commands.runEnd(() -> driverController.setRumble(RumbleType.kBothRumble, 1), () -> driverController.setRumble(RumbleType.kBothRumble, 0)).withTimeout(0.5)
                            );
                        }

                        if(commandingShootSpeaker) {
                            shootingStateSwitched = true;
                            currentShootingState = ShootingState.ALIGN_SPEAKER;
                        }

                        break;
                    case ALIGN_SPEAKER:
                        
                        if(shootingStateSwitched) {
                            shootingStateSwitched = false;
                            commandScheduler.schedule(
                                new SpeakerPositionCommand(armSubsystem, drivetrainSubsystem),
                                // Commands.runOnce(() -> {
                                //     double distFromWall = Math.sqrt(Math.pow(drivetrainSubsystem.getPose().getX() - (ON_RED_ALLIANCE.getAsBoolean() ? 16.46 : 0), 2));
                                //     double distFromShooterFront = distFromWall - .46;
                                //     double vertical = 2.11 - 0.30; // 2.11 is height from ground. 0.30 is average height of turbotake

                                //     double shooterAngle = Math.atan(vertical / distFromShooterFront);

                                // }, armSubsystem),
                                ledSubsystem.setPatternCommand(BlinkinPattern.COLOR_1_PATTERN_BREATH_FAST));
                            timer.reset();
                            timer.start();
                        }

                        if(commandingShootSpeaker && timer.hasElapsed(0.3)) {
                            shootingStateSwitched = true;
                            currentShootingState = ShootingState.SHOOT_SPEAKER;
                        }

                        if(shootingStateSwitched) {
                            timer.stop();
                        }

                        break;
                    case SHOOT_SPEAKER:
                        if(shootingStateSwitched) {
                            shootingStateSwitched = false;
                            turbotakeSubsystem.setIndexerPercent(1);
                            timer.reset();
                            timer.start();
                        }

                        double minLaunchSeconds = 0.5;
                        if(timer.hasElapsed(minLaunchSeconds)) {
                            shootingStateSwitched = true;
                            stateSwitched = true;
                            currentState = State.DRIVE_WITHOUT_PIECE;
                        }

                        if(shootingStateSwitched) {
                            timer.stop();
                        }
                        break;
                }

                if(commandingScoreAmp) {
                    currentState = State.SCORE_AMP;
                    stateSwitched = true;
                }

                // if(!turbotakeSubsystem.isPieceDetected() && turbotakeSubsystem.getIndexerVelocity() < 0) {
                //     stateSwitched = true;
                //     currentState = State.DRIVE_WITHOUT_PIECE;
                // }

                if(stateSwitched) {
                    commandScheduler.schedule(new StowPositionCommand(armSubsystem));
                    turbotakeSubsystem.setIndexerPercent(0);
                    turbotakeSubsystem.setShooterPercent(0);
                }

                break;
            case SCORE_AMP:

                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(
                        ledSubsystem.setPatternCommand(BlinkinPattern.COLOR_2_PATTERN_BREATH_FAST),
                        scoreAmpCommand
                    );
                }

                if(!commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITH_PIECE;
                }

                if(!turbotakeSubsystem.isPieceDetected() && turbotakeSubsystem.getIndexerVelocity() < 0) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;
                }

                if(stateSwitched) {
                    scoreAmpCommand.cancel();
                    commandScheduler.schedule(
                        new StowPositionCommand(armSubsystem),
                        Commands.runOnce( () -> turbotakeSubsystem.setIndexerPercent(0))
                    );
                }

                break;
            default:
                // The default case will only run if the currentState doesn't have a corresponding case in the switch statement. This is an error
                ledSubsystem.setPatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_LARSON_SCANNER_RED);
                DriverStation.reportError("TeleopStateMachine: state " + currentState.toString() + " behavior is undefined", false);
                break;
        }
    }

    private void updateTelemetry() {
        statePublisher.set(currentState.toString());
        shootingStatePublisher.set(currentShootingState.toString());
        pickupStatePublisher.set(currentPickupState.toString());
    }

    public Command pickupGroundCommand() {
        return Commands.runEnd(() -> commandingPickupGround = true, () -> commandingPickupGround = false);
    }

    public Command alignSpeakerCommand() {
        return Commands.runEnd(() -> commandingAlignSpeaker = true, () -> commandingAlignSpeaker = false);
    }

    public Command shootSpeakerCommand() {
        return Commands.runEnd(() -> commandingShootSpeaker = true, () -> commandingShootSpeaker = false);
    }

    public Command scoreAmpCommand() {
        return Commands.runEnd(() -> commandingScoreAmp = true, () -> commandingScoreAmp = false);
    }

}
