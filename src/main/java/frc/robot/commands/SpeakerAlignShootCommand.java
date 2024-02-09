package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.TurboTakeSubsystem;

public class SpeakerAlignShootCommand extends Command{
    DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    ArmSubsystem armSubsystem = new ArmSubsystem();
    TurboTakeSubsystem turboTakeSubsystem = new TurboTakeSubsystem();

    public SpeakerAlignShootCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, TurboTakeSubsystem turboTakeSubsystem) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.armSubsystem = armSubsystem;
        this.turboTakeSubsystem = turboTakeSubsystem;

        addRequirements(drivetrainSubsystem, armSubsystem, turboTakeSubsystem);
    }

    protected Command getCommand() {
        if (turboTakeSubsystem.pieceDetected()) //FIXME: Add function to check if robot is in shooting range
            return new SequentialCommandGroup( //FIXME: Add function to align robot to speaker and wait methods
                turboTakeSubsystem.runOnce(() -> turboTakeSubsystem.SetShooterSpeed(1)),
                armSubsystem.speakerPositionCommand(),
                turboTakeSubsystem.runOnce(() -> turboTakeSubsystem.SetIndexerSpeed(0.5)),
                turboTakeSubsystem.runOnce(() -> turboTakeSubsystem.SetShooterSpeed(0)),
                turboTakeSubsystem.runOnce(() -> turboTakeSubsystem.SetIndexerSpeed(0))
            );
        else
            return new SequentialCommandGroup(null);
    }
    
}
