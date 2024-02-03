package frc.robot.commands;

import static frc.robot.Constants.INDEXER_SPEED;
import static frc.robot.Constants.SHOOTER_SPEED;

import frc.robot.subsystems.*;


//import joint subsystem branch

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurboTakeSubsystem;

//import frc.robot.Constants;
 
public class IntakePieceCommand extends Command{
    public TurboTakeSubsystem turbotakesubsystem = new TurboTakeSubsystem();
    public boolean hasPiece;
    public long startTime;

    public IntakePieceCommand(TurboTakeSubsystem turbotakesubsystem){
        this.turbotakesubsystem = turbotakesubsystem;

        addRequirements(turbotakesubsystem);
    }

    @Override
    public void initialize(){
        hasPiece = false;
        startTime = RobotController.getFPGATime();
    }

    

    @Override
    public void execute(){
        if(turbotakesubsystem.pieceDetected()){
            System.out.println("We have a Piece");
            
        }
    }


    //States
    public void IdleState(){//Stowed position with LEDS at idle state
        //stowPositionCommand();
        //ledsubsystem.idlePatternCommand(IDLE_PATTERN);
    }

    public void intakeState(){//Intake position with intake motor running, go back to idle
        //intakePosititonCommand();
        turbotakesubsystem.SetIndexerSpeed(INDEXER_SPEED);
        //if(hasPiece == true)//ledsubsystem.idlePatternCommand(INTAKE_PATTERN);
        //stowPositionCommand();
    }

    public void shootState(){//amp/speaker position with shooter running until indexer runs moving the note
        turbotakesubsystem.SetShooterSpeed(SHOOTER_SPEED);
        //delay for x seconds
        //amp or speaker position
        turbotakesubsystem.SetIndexerSpeed(INDEXER_SPEED);
        //delay or a 1 second
        turbotakesubsystem.setIndexSpeed(0);
        turbotakesubsystem.SetShooterSpeed(0);
    }
}
