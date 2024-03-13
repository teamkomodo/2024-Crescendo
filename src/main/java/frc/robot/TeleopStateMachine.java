package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
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
import frc.robot.subsystems.ClimberSubsystem;
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
        ALIGN_AMP,
        SCORE_AMP,
        CLIMB,
        EJECT_PIECE,
        END
    }

    public static enum ShootingState {
        INACTIVE,
        PREPARE_SHOOT,
        READY_SHOOT,
        ALIGN_SPEAKER,
        SHOOT_SPEAKER,
    }

    public static enum PickupState {
        INACTIVE,
        NO_PIECE,
        PARTIAL_AQUISITION,
        ALIGN_PIECE
    }

    public static enum ClimbState {
        INACTIVE,
        EXTEND,
        READY,
        ASCEND,
        SCORE_TRAP
    }

    private static final boolean smartShooting = false;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("teleopstatemachine");
    private final StringPublisher statePublisher = table.getStringTopic("teleopstate").publish();
    private final StringPublisher shootingStatePublisher = table.getStringTopic("shootingstate").publish();
    private final StringPublisher pickupStatePublisher = table.getStringTopic("pickupstate").publish();

    private final BooleanEntry enabledEntry = table.getBooleanTopic("statemachineenabled").getEntry(true);

    // Store a reference to the Command Scheduler so it's easier to schedule commands
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private final Timer timer = new Timer();

    // Store references to each of the subsystems, so that we can schedule their commands.
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ArmSubsystem armSubsystem;
    private final TurbotakeSubsystem turbotakeSubsystem;
    private final LEDSubsystem ledSubsystem;
    private final ClimberSubsystem climberSubsystem;

    private final XboxController driverController;
    private final XboxController operatorController;

    private boolean enabled = true;

    // Store the current state
    private State currentState = State.START;
    private ShootingState currentShootingState = ShootingState.PREPARE_SHOOT;
    private PickupState currentPickupState = PickupState.INACTIVE;
    //private ClimbState currentClimbState = ClimbState.EXTEND;

    // This will turn to true when the current state's exit condition is met, and will signal the next state to run its entrance code
    private boolean stateSwitched = true;
    private boolean shootingStateSwitched = true;
    private boolean currentPickupStateSwitched = true;
    //private boolean climbStateSwitched = true;

    private boolean commandingPickupGround = false;
    private boolean commandingSpinUp = false;
    private boolean commandingShootSpeaker = false;
    private boolean commandingAlignAmp = false;
    private boolean commandingScoreAmp = false;
    private boolean commandingEject = false;
    private boolean commandingClimbExtend = false;
    private boolean commandingClimbAscend = false;

    public TeleopStateMachine(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, TurbotakeSubsystem turbotakeSubsystem, LEDSubsystem ledSubsystem, ClimberSubsystem climberSubsystem, XboxController driverController, XboxController operatorController) {

        // Set our references
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.turbotakeSubsystem = turbotakeSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.climberSubsystem = climberSubsystem;

        this.driverController = driverController;
        this.operatorController = operatorController;

        enabledEntry.set(enabled);

    }

    public void init() {
        currentState = State.START;
        stateSwitched = true;

        currentShootingState = ShootingState.INACTIVE;
        shootingStateSwitched = true;

        currentPickupStateSwitched = true;
        currentPickupState = PickupState.INACTIVE;

        commandingPickupGround = false;
        commandingShootSpeaker = false;
        commandingAlignAmp = false;
        commandingScoreAmp = false;
    }

    public void periodic() {

        updateTelemetry();

        if(!enabled)
            return;
        // Switch through all of all states to determine what code to execute this iteration
        switch (currentState) {
            case START:
                currentState = State.DRIVE_WITHOUT_PIECE; //turbotakeSubsystem.isPieceDetected()? State.DRIVE_WITH_PIECE : State.DRIVE_WITHOUT_PIECE;
                stateSwitched = true;
                commandScheduler.schedule(
                    new StowPositionCommand(armSubsystem),
                    Commands.runOnce(() -> turbotakeSubsystem.setShooterPercent(0)),
                    Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0))
                );
                // intentionally no break statement
            case DRIVE_WITHOUT_PIECE:

                // Did we just enter this state?
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(
                        ledSubsystem.setFramePatternCommand(BlinkinPattern.COLOR_1_PATTERN_BREATH_FAST),
                        ledSubsystem.setTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_RED)
                    );
                }


                // Check exit conditions
                if(commandingPickupGround) {
                    currentState = State.PICKUP_GROUND;
                    stateSwitched = true;
                }

                if(commandingEject) {
                    currentState = State.EJECT_PIECE;
                    stateSwitched = true;
                }

                if(commandingClimbExtend) {
                    stateSwitched = true;
                    currentState = State.CLIMB;
                }

                break;
            case DRIVE_WITH_PIECE:

                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(
                        ledSubsystem.setFramePatternCommand(BlinkinPattern.COLOR_2_PATTERN_BREATH_FAST),
                        ledSubsystem.setTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN),
                        new StowPositionCommand(armSubsystem)
                    );
                }

                // boolean pastMidline = (drivetrainSubsystem.getPose().getX() < 8.5 && !ON_RED_ALLIANCE.getAsBoolean()) || (drivetrainSubsystem.getPose().getX() > 8.5 && ON_RED_ALLIANCE.getAsBoolean());
                if(commandingSpinUp) {
                    stateSwitched = true;
                    currentState = State.SCORE_SPEAKER;
                }

                if(commandingAlignAmp) {
                    stateSwitched = true;
                    currentState = State.ALIGN_AMP;
                }

                if(commandingEject) {
                    stateSwitched = true;
                    currentState = State.EJECT_PIECE;
                }

                if(commandingClimbExtend) {
                    stateSwitched = true;
                    currentState = State.CLIMB;
                }

                break;
            case EJECT_PIECE:
                if(stateSwitched) {
                    stateSwitched = false;

                    commandScheduler.schedule(Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(-0.5)));
                    timer.restart();
                    commandingEject = false;
                }

                if(timer.hasElapsed(0.5)) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;

                    commandScheduler.schedule(Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0)));
                }

                break;
            case PICKUP_GROUND:

                if(stateSwitched) {
                    stateSwitched = false;
                    currentPickupState = PickupState.NO_PIECE;
                    currentPickupStateSwitched = true;
                    commandScheduler.schedule(
                        ledSubsystem.setFramePatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_LIGHT_CHASE_BLUE),
                        ledSubsystem.setTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_VIOLET)
                    );
                }

                // Internal State Machine

                final double indexerRampUpTime = 2.0;
                final double intakeCurrentThreshold = 15;

                switch (currentPickupState) {
                    case INACTIVE:
                        currentPickupState = PickupState.NO_PIECE;
                        currentPickupStateSwitched = true;
                    case NO_PIECE:

                        if(currentPickupStateSwitched) {
                            currentPickupStateSwitched = false;
                            timer.restart();
                            commandScheduler.schedule(
                                new IntakePositionCommand(armSubsystem),
                                Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(1.0))
                            );

                        }

                        if(!commandingPickupGround) {
                            stateSwitched = true;
                            currentState = State.DRIVE_WITHOUT_PIECE;
                        }

                        if(turbotakeSubsystem.isPieceDetected()) {
                            currentPickupStateSwitched = true;
                            currentPickupState = PickupState.ALIGN_PIECE;
                        }

                        if(timer.hasElapsed(indexerRampUpTime) && turbotakeSubsystem.getFilteredCurrent() - intakeCurrentThreshold > 0) {
                            currentPickupStateSwitched = true;
                            currentPickupState = PickupState.PARTIAL_AQUISITION;
                        }

                        break;
                    case PARTIAL_AQUISITION:

                        if(currentPickupStateSwitched) {
                            currentPickupStateSwitched = false;
                            commandScheduler.schedule(
                                new StowPositionCommand(armSubsystem),
                                flashGreenCommand(),
                                xboxRumbleCommand(driverController, 0.5),
                                xboxRumbleCommand(operatorController, 0.5)
                            );
                            turbotakeSubsystem.setIndexerPercent(0.3);
                        }

                        if(turbotakeSubsystem.isPieceDetected()) {
                            currentPickupStateSwitched = true;
                            currentPickupState = PickupState.ALIGN_PIECE;
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

                if(commandingEject) {
                    stateSwitched = true;
                    currentState = State.EJECT_PIECE;
                }

                if(stateSwitched) {
                    commandScheduler.schedule(new StowPositionCommand(armSubsystem));
                    currentPickupState = PickupState.INACTIVE;
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
                    case INACTIVE:
                        currentShootingState = ShootingState.PREPARE_SHOOT;
                        currentPickupStateSwitched = true;
                    case PREPARE_SHOOT:

                        double shooterThreshold = SHOOTER_SPEED - 200;

                        if(shootingStateSwitched) {
                            shootingStateSwitched = false;
                            turbotakeSubsystem.setShooterVelocity(SHOOTER_SPEED);
                            commandScheduler.schedule(ledSubsystem.setFramePatternCommand(BlinkinPattern.COLOR_1_PATTERN_LARSON_SCANNER));

                            if(!smartShooting) {
                                commandScheduler.schedule(
                                    new SpeakerPositionCommand(armSubsystem)
                                );
                            }
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
                                ledSubsystem.setTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_YELLOW),
                                xboxRumbleCommand(operatorController, 0.5)
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

                            if(smartShooting) {
                                commandScheduler.schedule(
                                    Commands.sequence(
                                        Commands.runOnce(() -> armSubsystem.setElevatorPosition(1)),
                                        Commands.run(() -> armSubsystem.setTurbotakeAngle(calculateShooterAngle()), armSubsystem).until(() -> shootingStateSwitched)
                                    ),
                                    drivetrainSubsystem.pointToSpeakerCommand()
                                );
                            }
                            timer.restart();
                        }

                        if(timer.hasElapsed(.3)) {
                            shootingStateSwitched = true;
                            currentShootingState = ShootingState.SHOOT_SPEAKER;
                        }

                        break;
                    case SHOOT_SPEAKER:
                        if(shootingStateSwitched) {
                            shootingStateSwitched = false;
                            turbotakeSubsystem.setIndexerPercent(1);
                            timer.restart();
                        }

                        double launchTime = 0.5;
                        if(timer.hasElapsed(launchTime)) {
                            shootingStateSwitched = true;
                            stateSwitched = true;
                            currentState = State.DRIVE_WITHOUT_PIECE;

                            commandScheduler.schedule(flashGreenCommand());
                        }

                        break;
                }

                if(commandingAlignAmp) {
                    currentState = State.ALIGN_AMP;
                    stateSwitched = true;
                }

                if(commandingEject) {
                    currentState = State.EJECT_PIECE;
                    stateSwitched = true;
                }

                if(stateSwitched) {
                    commandScheduler.schedule(new StowPositionCommand(armSubsystem));
                    turbotakeSubsystem.setIndexerPercent(0);
                    turbotakeSubsystem.setShooterPercent(0);

                    currentShootingState = ShootingState.INACTIVE;
                }

                break;
            case ALIGN_AMP:
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(
                        new AmpPositionCommand(armSubsystem)
                    );
                    timer.restart();
                    // new WaitCommand(2);
                    // stateSwitched = true;
                    // currentState = State.SCORE_AMP;
                }

                if(timer.hasElapsed(2) && commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.SCORE_AMP;
                }

                if(!commandingAlignAmp) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITH_PIECE;
                    commandScheduler.schedule(new StowPositionCommand(armSubsystem));
                }
                break;
            case SCORE_AMP:

                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(
                        ledSubsystem.setFramePatternCommand(BlinkinPattern.COLOR_2_PATTERN_BREATH_FAST),
                        Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(-1.0))
                    );
                    timer.restart();
                }

                if(timer.hasElapsed(0.5)) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;
                }

                if(stateSwitched) {
                    commandScheduler.schedule(
                        new StowPositionCommand(armSubsystem),
                        Commands.runOnce(() -> turbotakeSubsystem.setIndexerPercent(0))
                    );
                }

                break;
            case CLIMB:

                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(
                        ledSubsystem.setFramePatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_BEATS_PER_MINUTE_PARTY_PALETTE),
                        new IntakePositionCommand(armSubsystem)
                    );
                }

                break;
            case END:
                
                break;
            default:
                // The default case will only run if the currentState doesn't have a corresponding case in the switch statement. This is an error
                ledSubsystem.setFramePatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_LARSON_SCANNER_RED);
                DriverStation.reportError("TeleopStateMachine: state " + currentState.toString() + " behavior is undefined", false);
                break;
        }
    }

    private void updateTelemetry() {
        statePublisher.set(currentState.toString());
        shootingStatePublisher.set(currentShootingState.toString());
        pickupStatePublisher.set(currentPickupState.toString());

        boolean newEnabled = enabledEntry.get();
        if(newEnabled != enabled) {
            System.out.println("Change state machine");
            setEnabled(newEnabled);
        }
    }

    private Rotation2d calculateShooterAngle() {
        double distFromWall = Math.sqrt(Math.pow(drivetrainSubsystem.getPose().getX() - (ON_RED_ALLIANCE.getAsBoolean() ? 16.46 : 0), 2));
        double distFromShooterFront = distFromWall - .46;
        double vertical = 2.11 - 0.30; // 2.11 is height from ground. 0.30 is average height of turbotake

        double shooterAngle = Math.atan(vertical / distFromShooterFront);

        return Rotation2d.fromRadians(shooterAngle);
    }

    public void setEnabled(boolean enabled) {
        this.enabled = enabled;
        if(this.enabled) {
            currentState = State.START;
        }else {
            commandScheduler.schedule(
                ledSubsystem.setTurbotakePatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_1_ON_COLOR_2),
                ledSubsystem.setFramePatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_2_ON_COLOR_1)
            );
        }
    }

    public boolean isEnabled() {
        return enabled;
    }

    private Command xboxRumbleCommand(XboxController controller, double time) {
        return Commands.runEnd(() -> controller.setRumble(RumbleType.kLeftRumble, 1), () -> controller.setRumble(RumbleType.kLeftRumble, 0)).withTimeout(time);
    }

    private Command flashGreenCommand() {
        return Commands.sequence(
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1),
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_GREEN).withTimeout(0.1),
            ledSubsystem.setTempTurbotakePatternCommand(BlinkinPattern.SOLID_COLORS_BLACK).withTimeout(0.1)
        );
    }

    public Command pickupGroundCommand() {
        return Commands.runEnd(() -> commandingPickupGround = true, () -> commandingPickupGround = false);
    }

    public Command spinUpCommand() {
        return Commands.runEnd(() -> commandingSpinUp = true, () -> commandingSpinUp = false);
    }

    public Command shootSpeakerCommand() {
        return Commands.runEnd(() -> commandingShootSpeaker = true, () -> commandingShootSpeaker = false);
    }

    public Command alignAmpCommand() {
        return Commands.runEnd(() -> commandingAlignAmp = true, () -> commandingAlignAmp = false);
    }

    public Command scoreAmpCommand() {
        return Commands.runEnd(() -> commandingScoreAmp = true, () -> commandingScoreAmp = false);
    }

    public Command ejectCommand() {
        return Commands.runEnd(() -> commandingEject = true, () -> commandingEject = false);
    }

    public Command extendClimbCommand() {
        return Commands.runEnd(() -> commandingClimbExtend = true, () -> commandingClimbExtend = false);
    }

    public Command ascendClimbCommand() {
        return Commands.runEnd(() -> commandingClimbAscend = true, () -> commandingClimbAscend = false);
    }

}
