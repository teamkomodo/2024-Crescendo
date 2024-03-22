package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ProfiledClimbCommand extends Command{

    private final ClimberSubsystem climberSubsystem;
    private final double velocity;

    private double desiredPosition = 0;

    /**
     * @param climberSubsystem
     * @param velocity velocity in rev/s
     */
    public ProfiledClimbCommand(ClimberSubsystem climberSubsystem, double velocity) {
        this.climberSubsystem = climberSubsystem;
        this.velocity = velocity;
    }

    @Override
    public void initialize() {
        desiredPosition = climberSubsystem.getLeftMotorPosition();
    }

    @Override
    public void execute() {
        desiredPosition += velocity * 0.02;
        climberSubsystem.setClimberPosition(desiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.holdClimberPosition();
    }
    
}
