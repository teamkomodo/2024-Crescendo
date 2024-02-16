package frc.robot.commands.States;

import static frc.robot.Constants.INDEXER_SPEED;
import static frc.robot.Constants.SPEAKER_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.TurboTakeSubsystem;


public class ShootSpeakerState extends Command{
    public TurboTakeSubsystem turbotakeSubsystem = new TurboTakeSubsystem();

    public ShootSpeakerState(TurboTakeSubsystem turboTakeSubsystem){
        addRequirements(turbotakeSubsystem);
    }

    protected Command getCommand(){

        return new SequentialCommandGroup(
            turbotakeSubsystem.runOnce(() -> turbotakeSubsystem.setShooterVelocity(SPEAKER_SPEED)),
            new WaitCommand(2),
            turbotakeSubsystem.runOnce(() -> turbotakeSubsystem.setIndexerVelocity(INDEXER_SPEED)),
            new WaitCommand(2),
            turbotakeSubsystem.runOnce(() -> turbotakeSubsystem.setIndexerVelocity(0)),
            turbotakeSubsystem.runOnce(() -> turbotakeSubsystem.setShooterVelocity(0))
        );    
    }
}
