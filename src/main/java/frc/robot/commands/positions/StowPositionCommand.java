package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ArmSubsystem;

public class StowPositionCommand extends DynamicCommand {

    private final ArmSubsystem armSubsystem;

    public StowPositionCommand(ArmSubsystem armSubsystem) {
        
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(!(armSubsystem.isJointZeroed() || armSubsystem.isElevatorZeroed())) {
            return Commands.sequence(
                new StowPositionCommand(armSubsystem)
            );
        }
        
        if (armSubsystem.getJointPosition() < 2.5) {
            return new SequentialCommandGroup(
                armSubsystem.jointPositionCommand(16),
                new WaitCommand(0.7),
                armSubsystem.elevatorZeroPositionCommand(),
                new WaitCommand(0.1),
                armSubsystem.jointStowPositionCommand(),
                armSubsystem.elevatorStowPositionCommand()
            );
        }
        return new SequentialCommandGroup(
            armSubsystem.jointPositionCommand(16),
            new WaitCommand(0.1),
            armSubsystem.jointStowPositionCommand(),
            armSubsystem.elevatorStowPositionCommand()
        );
    }
}