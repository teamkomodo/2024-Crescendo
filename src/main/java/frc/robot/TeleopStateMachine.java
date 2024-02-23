package frc.robot;

import static frc.robot.Constants.SHOOTER_MAX_VELOCITY;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PickupCommand;
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
        PREPARE_SHOOT,
        READY_SHOOT,
        ALIGN_SPEAKER,
        SHOOT_SPEAKER,
        SCORE_AMP
    }

    private final StringPublisher statePublisher = NetworkTableInstance.getDefault().getStringTopic("teleopstate").publish();

    // Store a reference to the Command Scheduler so it's easier to schedule commands
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private final Timer timer = new Timer();
    
    // Store references to each of the subsystems, so that we can schedule their commands.
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ArmSubsystem armSubsystem;
    private final TurbotakeSubsystem turbotakeSubsystem;
    private final LEDSubsystem ledSubsystem;

    // Store the current state
    private State currentState = State.START;

    // This will turn to true when the current state's exit condition is met, and will signal the next state to run its entrance code
    private boolean stateSwitched = false;

    private boolean commandingPickupGround = false;
    private boolean commandingAlignSpeaker = false;
    private boolean commandingShootSpeaker = false;
    private boolean commandingScoreAmp = false;

    public TeleopStateMachine(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, TurbotakeSubsystem turbotakeSubsystem, LEDSubsystem ledSubsystem) {

        // Set our references
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.turbotakeSubsystem = turbotakeSubsystem;
        this.ledSubsystem = ledSubsystem;

        // Run the periodic method every iteration by wrapping it in a command and scheduling it
        // This is the same way we can schedule other commands in the periodic method
        // this::periodic passes the periodic() method itself as the argument for Commands.run(), which will later invoke it once each iteration
        commandScheduler.schedule(Commands.run(this::periodic));

    }

    private void periodic() {

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

                if(drivetrainSubsystem.getPose().getX() < 8.5) { // TODO Account for alliance
                    stateSwitched = true;
                    currentState = State.PREPARE_SHOOT;
                }

                if(commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.SCORE_AMP;
                }

                break;
            case PICKUP_GROUND:
                
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(Commands.sequence(
                        new IntakePositionCommand(armSubsystem),
                        new PickupCommand(armSubsystem, turbotakeSubsystem))
                    , ledSubsystem.setPatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_COLOR_WAVES_FOREST_PALETTE));
                }

                if(!commandingPickupGround) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;
                }

                if(turbotakeSubsystem.isPieceDetected()) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITH_PIECE;
                }

                if(stateSwitched) {
                    commandScheduler.schedule(new StowPositionCommand(armSubsystem));
                    turbotakeSubsystem.setIndexerPercent(0);
                }
                
                break;
            case PREPARE_SHOOT:

                double shooterThreshold = SHOOTER_MAX_VELOCITY - 50;

                if(stateSwitched) {
                    stateSwitched = false;
                    turbotakeSubsystem.setShooterVelocity(SHOOTER_MAX_VELOCITY);
                    commandScheduler.schedule(ledSubsystem.setPatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_BREATH_BLUE));
                }

                if(turbotakeSubsystem.getShooterVelocity() > shooterThreshold) {
                    stateSwitched = true;
                    currentState = State.READY_SHOOT;
                }

                if(commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.SCORE_AMP;
                }

                if(stateSwitched) {
                    turbotakeSubsystem.setShooterPercent(0);
                }

                break;
            case READY_SHOOT:
                
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(ledSubsystem.setPatternCommand(BlinkinPattern.SOLID_COLORS_GREEN));
                }

                if(commandingAlignSpeaker) {
                    stateSwitched = true;
                    currentState = State.ALIGN_SPEAKER;
                }
                
                if(commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.SCORE_AMP;
                }

                break;
            case ALIGN_SPEAKER:
                
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(new SpeakerPositionCommand(armSubsystem), ledSubsystem.setPatternCommand(BlinkinPattern.COLOR_1_PATTERN_BREATH_FAST));
                }

                if(commandingShootSpeaker) {
                    stateSwitched = true;
                    currentState = State.SHOOT_SPEAKER;
                }

                break;
            case SHOOT_SPEAKER:
                if(stateSwitched) {
                    stateSwitched = false;
                    turbotakeSubsystem.setIndexerPercent(1);
                    timer.reset();
                    timer.start();
                }

                double minLaunchSeconds = 0.5;
                if(!turbotakeSubsystem.isPieceDetected() && timer.hasElapsed(minLaunchSeconds)) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;
                }

                if(stateSwitched) {
                    turbotakeSubsystem.setIndexerPercent(0);
                    turbotakeSubsystem.setShooterPercent(0);
                    timer.stop();
                }
                break;
            case SCORE_AMP:
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(new AmpPositionCommand(armSubsystem), ledSubsystem.setPatternCommand(BlinkinPattern.COLOR_2_PATTERN_BREATH_FAST));
                }

                if(!commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITH_PIECE;
                }

                if(stateSwitched) {
                    commandScheduler.schedule(new StowPositionCommand(armSubsystem));
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
        statePublisher.set(currentState.name());
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
