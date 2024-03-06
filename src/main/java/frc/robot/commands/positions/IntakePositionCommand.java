package frc.robot.commands.positions;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ArmSubsystem;

public class IntakePositionCommand extends DynamicCommand{

    private final ArmSubsystem armSubsystem;

    public IntakePositionCommand(ArmSubsystem armSubsystem) {
        
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    protected Command getCommand() {
        if(!(armSubsystem.isJointZeroed() || armSubsystem.isElevatorZeroed())) {
            return Commands.sequence(
                Commands.parallel(
                    armSubsystem.jointZeroCommand(),
                    armSubsystem.elevatorZeroCommand()),
                new SequentialCommandGroup(
                    armSubsystem.elevatorZeroPositionCommand(),
                    new WaitCommand(0.3),
                    armSubsystem.jointStowPositionCommand(),
                    armSubsystem.elevatorIntakePositionCommand(),
                    new WaitCommand(0.5),
                    armSubsystem.jointIntakePositionCommand()
                )
            );
        }

        if (armSubsystem.getCommandedPosition() == "intake") {
            return null;
        }
        
        return new SequentialCommandGroup(
            armSubsystem.elevatorZeroPositionCommand(),
            new WaitCommand(0.1),
            armSubsystem.jointStowPositionCommand(),
            armSubsystem.elevatorIntakePositionCommand(),
            new WaitCommand(0.4),
            armSubsystem.jointIntakePositionCommand()
        );
    }
}