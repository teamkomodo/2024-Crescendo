package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

public class SpeakerPositionCommand extends Command{

    private final ArmSubsystem armSubsystem;

    public SpeakerPositionCommand(ArmSubsystem armSubsystem) {
        
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    protected Command getCommand() {
        if(!(armSubsystem.isZeroed())) {
            return Commands.parallel(
                armSubsystem.jointZeroCommand(),
                armSubsystem.elevatorZeroCommand()
            );
        }
        return new SequentialCommandGroup(
            armSubsystem.jointSpeakerPositionCommand(),
            armSubsystem.elevatorSpeakerPositionCommand()
        );
    }
}