package frc.robot;

import static frc.robot.Constants.SHOOTER_MAX_VELOCITY;

import edu.wpi.first.wpilibj.DriverStation;
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
import frc.robot.subsystems.TurbotakeSubsystem;

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

    // Store a reference to the Command Scheduler so it's easier to schedule commands
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    
    // Store references to each of the subsystems, so that we can schedule their commands.
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final ArmSubsystem armSubsystem;
    private final TurbotakeSubsystem turbotakeSubsystem;

    // Store the current state
    private State currentState = State.START;

    // This will turn to true when the current state's exit condition is met, and will signal the next state to run its entrance code
    private boolean stateSwitched = false;

    private boolean commandingPickupGround = false;
    private boolean commandingAlignSpeaker = false;
    private boolean commandingShootSpeaker = false;
    private boolean commandingScoreAmp = false;

    public TeleopStateMachine(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, TurbotakeSubsystem turbotakeSubsystem) {

        // Set our references
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.turbotakeSubsystem = turbotakeSubsystem;

        // Run the periodic method every iteration by wrapping it in a command and scheduling it
        // This is the same way we can schedule other commands in the periodic method
        // this::periodic passes the periodic() method itself as the argument for Commands.run(), which will later invoke it once each iteration
        commandScheduler.schedule(Commands.run(this::periodic));

    }

    private void periodic() {
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
                    );
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
                    commandScheduler.schedule(new SpeakerPositionCommand(armSubsystem));
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
                }

                if(!turbotakeSubsystem.isPieceDetected()) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;
                }

                if(stateSwitched) {
                    turbotakeSubsystem.setIndexerPercent(0);
                    turbotakeSubsystem.setShooterPercent(0);
                }
                break;
            case SCORE_AMP:
                if(stateSwitched) {
                    stateSwitched = false;
                    commandScheduler.schedule(new AmpPositionCommand(armSubsystem));
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
                DriverStation.reportError("TeleopStateMachine: state " + currentState.toString() + " behavior is undefined", false);
                break;
        }
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
