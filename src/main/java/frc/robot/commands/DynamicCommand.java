package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public abstract class DynamicCommand extends Command{

    protected Command command;

    @Override
    public void execute() {
        if(command == null)
            return;
        command.execute();
    }

    @Override
    public boolean isFinished() {
        if(command == null)
            return true;
        return command.isFinished();
    }

    @Override
    public void initialize() {
        command = getCommand();
        if(command == null)
            command = new InstantCommand();
        command.initialize();
    }

    protected abstract Command getCommand();

}