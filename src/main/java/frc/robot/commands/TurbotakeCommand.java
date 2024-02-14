package frc.robot.commands;

import static frc.robot.Constants.INDEXER_SPEED;
import static frc.robot.Constants.SHOOTER_SPEED;

//import frc.robot.subsystems.*;


//import joint subsystem branch

import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.TurboTakeSubsystem;

//import frc.robot.Constants;
 
public class TurbotakeCommand extends Command{
    public TurboTakeSubsystem turbotakesubsystem = new TurboTakeSubsystem();
    public boolean hasPiece;
    public long startTime;
    

    public TurbotakeCommand(TurboTakeSubsystem turbotakesubsystem){
        this.turbotakesubsystem = turbotakesubsystem;

        addRequirements(turbotakesubsystem);
    }

    @Override
    public void initialize(){
        hasPiece = false;
        startTime = RobotController.getFPGATime();
    }

    

    // @Override
    // public void execute(){
        
    // }


    //States
    public void IdleState(){//Stowed position with LEDS at idle state
        //armSubsystem.stowPositionCommand();
        //ledSubsystem.idlePatternCommand(IDLE_PATTERN);
    }

    public void intakeState(){//Intake position with intake motor running, go back to idle
        //armSubsystem.intakePosititonCommand();
        turbotakesubsystem.setIndexerVelocity(INDEXER_SPEED);
        new WaitCommand(3);
        turbotakesubsystem.setIndexerVelocity(0);
        if(turbotakesubsystem.pieceDetected()){
            hasPiece = true;
            System.out.println("We have a Piece: " + hasPiece);
        }
        new WaitCommand(2);
        //armSubsystem.stowPositionCommand();
    }

    public void shootAmpState(){//amp/speaker position with shooter running until indexer runs moving the note
        
        //armSubsystem.AmpPositionCommand();
        //ledSubsystem.shootPatternCommand(AMP_PATTERN);
        turbotakesubsystem.setShooterVelocity(SHOOTER_SPEED);
        new WaitCommand(3);
        turbotakesubsystem.setIndexerVelocity(INDEXER_SPEED);
        new WaitCommand(0.5);
        turbotakesubsystem.setIndexerVelocity(0);
        turbotakesubsystem.setShooterVelocity(0);
        IdleState();
    }

    public void shootSpeakerState(){
        //armSubsystem.SpeakerPositionCommand();
        turbotakesubsystem.setShooterVelocity(SHOOTER_SPEED);
        new WaitCommand(3);
        turbotakesubsystem.setIndexerVelocity(INDEXER_SPEED);
        new WaitCommand(0.5);
        turbotakesubsystem.setIndexerVelocity(0);
        turbotakesubsystem.setShooterVelocity(0);
        IdleState();
    }
}
