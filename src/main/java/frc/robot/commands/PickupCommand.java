package frc.robot.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TurbotakeSubsystem;

public class PickupCommand extends Command{
    
    private final ArmSubsystem armSubsystem;
    private final TurbotakeSubsystem turbotakeSubsystem;

    private final double rampUpTimeMicro = 0.5 * 1e6;
    private final double currentThreshold = 5;

    private long startTimeMicro = 0;
    private boolean finished = false;

    public PickupCommand(ArmSubsystem armSubsystem, TurbotakeSubsystem turbotakeSubsystem) {
        addRequirements(turbotakeSubsystem, armSubsystem);

        this.armSubsystem = armSubsystem;
        this.turbotakeSubsystem = turbotakeSubsystem;
    }

    @Override
    public void initialize() {
        turbotakeSubsystem.setIndexerPercent(1);
        startTimeMicro = RobotController.getFPGATime();
        finished = false;
    }

    @Override
    public void execute() {

        if(RobotController.getFPGATime() - startTimeMicro < rampUpTimeMicro)
            return;

        if(turbotakeSubsystem.getFilteredCurrent() - currentThreshold > 0) {
            armSubsystem.setJointPosition(armSubsystem.getJointPosition() + 3);
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

}
