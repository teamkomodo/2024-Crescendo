package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
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
        ALIGN_SHOOT,
        SCORE_AMP
    }

    // Store the current state
    private State currentState = State.START;

    // This will turn to true when the current state's exit condition is met, and will signal the next state to run its entrance code
    private boolean stateSwitched = false;

    // Store a reference to the Command Scheduler so it's easier to schedule commands
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();

    // Store a reference to the driver's Xbox controller
    private final XboxController driverController;
    
    // Store references to each of the subsystems, so that we can schedule their commands.
    private final DrivetrainSubsystem drivetrainSubsystem;

    public TeleopStateMachine(XboxController driverController, DrivetrainSubsystem drivetrainSubsystem) {

        // Set our references
        this.driverController = driverController;
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

                // Code to be executed each time this state runs

                // Check exit conditions
                if(driverController.getAButton()) {
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
                break;
            case PICKUP_GROUND:
                break;
            case PREPARE_SHOOT:
                break;
            case READY_SHOOT:
                break;
            case ALIGN_SHOOT:
                break;
            case SCORE_AMP:
                break;
            default:
                // The default case will only run if the currentState doesn't have a corresponding case in the switch statement. This is an error
                DriverStation.reportError("TeleopStateMachine: state " + currentState.toString() + " behavior is undefined", false);
                break;
        }
    }

}
