package frc.robot.commands.positions;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SpeakerPositionCommand extends DynamicCommand {

    private final ArmSubsystem armSubsystem;
    private final DrivetrainSubsystem drivetrainSubsystem;

    public SpeakerPositionCommand(ArmSubsystem armSubsystem, DrivetrainSubsystem drivetrainSubsystem) {
        
        this.armSubsystem = armSubsystem;
        this.drivetrainSubsystem = drivetrainSubsystem;

    }

    @Override
    protected Command getCommand() {
        if(!(armSubsystem.isJointZeroed() || armSubsystem.isElevatorZeroed())) {
            return Commands.sequence(
                Commands.parallel(
                    armSubsystem.jointZeroCommand(),
                    armSubsystem.elevatorZeroCommand()
                ),
                new SpeakerPositionCommand(armSubsystem, drivetrainSubsystem)
            );
        }

        return new SequentialCommandGroup(
            armSubsystem.elevatorZeroPositionCommand(),
            new WaitCommand(0.1),
            alignTurbotakeCommand()
        );
    }

    private Command alignTurbotakeCommand() {

        // TODO account for alliance
        double xDistance = drivetrainSubsystem.getPose().getX();
        double yDistance = 1.30;

        double angle = Math.atan(yDistance/xDistance);

        return Commands.runOnce(() -> armSubsystem.setTurbotakeAngle(Rotation2d.fromRadians(angle)));
    }
}