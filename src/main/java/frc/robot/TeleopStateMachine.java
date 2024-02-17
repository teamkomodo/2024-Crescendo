package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TeleopStateMachine {
    
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

    private State currentState = State.START;
    private boolean stateSwitched = false;
    private CommandScheduler commandScheduler = CommandScheduler.getInstance();

    public void periodic() {
        switch (currentState) {
            case START:
                // currentState = turbotakeSubsystem.hasPiece()? State.DRIVE_WITH_PIECE : State.DRIVE_WITHOUT_PIECE;
                stateSwitched = true;
            case DRIVE_WITHOUT_PIECE:

                if(stateSwitched) {
                    stateSwitched = false;
                    // entrance code
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
                DriverStation.reportError("TeleopStateMachine: state " + currentState.toString() + " behavior is undefined", false);
                break;
        }
    }

}
