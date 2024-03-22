package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class OneSideProfiledClimbCommand extends Command{

    private final ClimberSubsystem climberSubsystem;
    private final double velocity;
    private final boolean leftSide;

    private double desiredPosition = 0;

    /**
     * @param climberSubsystem
     * @param velocity velocity in rev/s
     */
    public OneSideProfiledClimbCommand(ClimberSubsystem climberSubsystem, double velocity, boolean leftSide) {
        this.climberSubsystem = climberSubsystem;
        this.velocity = velocity;
        this.leftSide = leftSide;
    }

    @Override
    public void initialize() {
        if(leftSide)
            desiredPosition = climberSubsystem.getLeftMotorPosition();
        else
            desiredPosition = climberSubsystem.getRightMotorPosition();
    }

    @Override
    public void execute() {
        desiredPosition += velocity * 0.02;
        if(leftSide)
            climberSubsystem.setLeftMotorPosition(desiredPosition);
        else
            climberSubsystem.setRightMotorPosition(desiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
