package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.JointSubsystem;

public class TrapPositionCommand extends Command{

    private final JointSubsystem jointSubsystem;

    public TrapPositionCommand(JointSubsystem jointSubsystem) {
        
        this.jointSubsystem = jointSubsystem;

        addRequirements(jointSubsystem);
    }

    protected Command getCommand() {
        if(!(jointSubsystem.isZeroed())) {
            return Commands.parallel(
                jointSubsystem.jointZeroCommand(),
                jointSubsystem.elevatorZeroCommand()
            );
        }
        return new SequentialCommandGroup(
            jointSubsystem.jointTrapPositionCommand(),
            jointSubsystem.elevatorTrapPositionCommand()
        );
    }
}