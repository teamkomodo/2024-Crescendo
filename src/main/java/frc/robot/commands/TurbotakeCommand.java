package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.TurboTakeSubsystem;


 
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
}
