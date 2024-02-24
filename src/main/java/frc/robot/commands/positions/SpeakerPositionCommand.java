package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ArmSubsystem;

public class SpeakerPositionCommand extends DynamicCommand{

    private final ArmSubsystem armSubsystem;

    public SpeakerPositionCommand(ArmSubsystem armSubsystem) {
        
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    protected Command getCommand() {
        if(!(armSubsystem.isJointZeroed() || armSubsystem.isElevatorZeroed())) {
            return Commands.parallel(
                armSubsystem.jointZeroCommand(),
                armSubsystem.elevatorZeroCommand()
            );
        }

        if (armSubsystem.getCommandedPosition() == "speaker") {
            return null;
        }

        if (armSubsystem.getJointPosition() < 2.5) {
            return new SequentialCommandGroup(
                armSubsystem.jointSpeakerPositionCommand(),
                new WaitCommand(0.2),
                armSubsystem.elevatorZeroPositionCommand(),
                armSubsystem.jointSpeakerPositionCommand(),
                armSubsystem.elevatorSpeakerPositionCommand()
            );
        }
        return new SequentialCommandGroup(
            armSubsystem.elevatorZeroPositionCommand(),
            new WaitCommand(0.1),
            armSubsystem.jointSpeakerPositionCommand(),
            armSubsystem.elevatorSpeakerPositionCommand()
        );
    }
}