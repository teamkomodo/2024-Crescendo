package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.JointSubsystem;

public class AmpPositionCommand extends Command{

    private final JointSubsystem jointSubsystem;

    public AmpPositionCommand(JointSubsystem jointSubsystem) {
        
        this.jointSubsystem = jointSubsystem;

        addRequirements(jointSubsystem);
    }

    protected Command getCommand() {
        if(!(jointSubsystem.isZeroed())) {
            return Commands.parallel(
                jointSubsystem.zeroCommand()
            );
        }
        return new SequentialCommandGroup(
            jointSubsystem.ampPositionCommand()

        );
    }
}