package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.JointSubsystem;

public class SpeakerPositionCommand extends Command{

    private final JointSubsystem jointSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    public SpeakerPositionCommand(JointSubsystem jointSubsystem, ElevatorSubsystem elevatorSubsystem) {
        
        this.jointSubsystem = jointSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;

        addRequirements(jointSubsystem, elevatorSubsystem);
    }

    protected Command getCommand() {
        if(!(jointSubsystem.isZeroed() && elevatorSubsystem.isZeroed())) {
            return Commands.parallel(
                jointSubsystem.zeroCommand(),
                elevatorSubsystem.zeroCommand()
            );
        }
        return new SequentialCommandGroup(
            jointSubsystem.speakerPositionCommand(),
            elevatorSubsystem.speakerPositionCommand()
        );
    }
}