package frc.robot.commands.positions;

import static frc.robot.Constants.JOINT_STOW_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ArmSubsystem;

public class AmpPositionCommand extends DynamicCommand {

    private final ArmSubsystem armSubsystem;

    public AmpPositionCommand(ArmSubsystem armSubsystem) {
        
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(!(armSubsystem.isJointZeroed() || armSubsystem.isElevatorZeroed())) {
            return Commands.sequence(
                Commands.parallel(
                    armSubsystem.jointZeroCommand(),
                    armSubsystem.elevatorZeroCommand()
                ),
                new AmpPositionCommand(armSubsystem)
            );
        }

        if (armSubsystem.getJointPosition() < JOINT_STOW_POSITION - 2) {
            return new SequentialCommandGroup(
                armSubsystem.jointStowPositionCommand(),
                new WaitCommand(0.1),
                armSubsystem.elevatorZeroPositionCommand(),
                new WaitCommand(0.1),
                armSubsystem.jointAmpPositionCommand(),
                armSubsystem.elevatorAmpPositionCommand()
            );
        }

        return new SequentialCommandGroup(
            armSubsystem.elevatorZeroPositionCommand(),
            new WaitCommand(0.1),
            armSubsystem.jointAmpPositionCommand(),
            armSubsystem.elevatorAmpPositionCommand()
        );
    }
}