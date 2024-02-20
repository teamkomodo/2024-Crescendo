package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DrivetrainSubsystem;

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

    // Store the current state
    private State currentState = State.START;

    // This will turn to true when the current state's exit condition is met, and will signal the next state to run its entrance code
    private boolean stateSwitched = false;

    private boolean commandingPickupGround = false;
    private boolean commandingAlignSpeaker = false;
    private boolean commandingShootSpeaker = false;
    private boolean commandingScoreAmp = false;

    public TeleopStateMachine(DrivetrainSubsystem drivetrainSubsystem) {

        // Set our references
        this.drivetrainSubsystem = drivetrainSubsystem;

        // Run the periodic method every iteration by wrapping it in a command and scheduling it
        // This is the same way we can schedule other commands in the periodic method
        // this::periodic passes the periodic() method itself as the argument for Commands.run(), which will later invoke it once each iteration
        commandScheduler.schedule(Commands.run(this::periodic));

    }

    private void periodic() {
        // Switch through all of all states to determine what code to execute this iteration
        switch (currentState) {
            case START:
                // currentState = turbotakeSubsystem.hasPiece()? State.DRIVE_WITH_PIECE : State.DRIVE_WITHOUT_PIECE;
                stateSwitched = true;
            case DRIVE_WITHOUT_PIECE:

                // Did we just enter this state?
                if(stateSwitched) {
                    stateSwitched = false;
                    
                    // Code to be executed when entering this state
                    if(drivetrainSubsystem.getCurrentCommand() != drivetrainSubsystem.getDefaultCommand()) {
                        drivetrainSubsystem.getCurrentCommand().cancel();
                    }
                }


                // Check exit conditions
                if(commandingPickupGround) {
                    // The a button was pressed (ie. one of the exit conditions was met), so we will switch the state to the PICKUP_GROUND state
                    currentState = State.PICKUP_GROUND;

                    // Signal to the next state that it needs to run its entrance code
                    stateSwitched = true;
                }
                
                // If stateSwitched is true, then we're planning on exiting this state after this iteration
                if(stateSwitched) {

                    // Code to be executed when exting this state

                }

                break;
            case DRIVE_WITH_PIECE:
                
                if(stateSwitched) {
                    stateSwitched = false;
                }

                if(drivetrainSubsystem.getPose().getX() < 8) { // TODO Measure and fix
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

                    // ground position command
                    // run intake command
                }

                if(!commandingPickupGround) {
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;
                }

                // has piece
                if(false == false) { // TODO turbotakeSubsystem.hasPiece();
                    stateSwitched = true;
                    currentState = State.DRIVE_WITH_PIECE;
                }

                if(stateSwitched) {
                    // stow position command
                    // stop intake command
                }
                
                break;
            case PREPARE_SHOOT:

                if(stateSwitched) {
                    stateSwitched = false;
                    // turbotake ramp up command
                }

                if(false == false) { // TODO turbotakeSubsystem.readyToShoot();
                    stateSwitched = true;
                    currentState = State.READY_SHOOT;
                }

                if(commandingScoreAmp) {
                    stateSwitched = true;
                    currentState = State.SCORE_AMP;
                }

                if(stateSwitched) {
                    // turbotake cancel ramp
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
                    // align to speaker command
                }

                if(commandingShootSpeaker) {
                    stateSwitched = true;
                    currentState = State.SHOOT_SPEAKER;
                }

                break;
            case SHOOT_SPEAKER:
                if(stateSwitched) {
                    stateSwitched = false;
                    // turbotake shoot command
                }

                if(false == false) { // !TODO turbotake.hasPiece()
                    stateSwitched = true;
                    currentState = State.DRIVE_WITHOUT_PIECE;
                }
                break;
            case SCORE_AMP:
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
