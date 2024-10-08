package frc.robot.commands.positions;

import static frc.robot.Constants.JOINT_STOW_POSITION;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DynamicCommand;
import frc.robot.subsystems.ArmSubsystem;

//TODO: tell the arm subsystem to move and stuff
public class DistanceSpeakerPositionCommand extends DynamicCommand {
    
    private final  ArmSubsystem armSubsystem;

    public DistanceSpeakerPositionCommand(ArmSubsystem armSubsystem){
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    protected Command getCommand(){

        return new SequentialCommandGroup(null);
    }



    
    
}
