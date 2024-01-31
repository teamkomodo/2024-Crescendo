package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TurboTakeSubsystem;

 
public class IntakePieceCommand extends Command{
    public TurboTakeSubsystem turbotakesubsystem = new TurboTakeSubsystem();
    private boolean hasPiece;
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
    public void end(boolean interrupted){
        if(!hasPiece){
            turbotakesubsystem.setMotorDutyCycle(0);
        }
    }

    @Override
    public void execute(){
        if(turbotakesubsystem.pieceDetected()){
            System.out.println("We have a Piece");
        }
    }
}
