package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class OffsetClimbCommand extends Command{

    private final ClimberSubsystem climberSubsystem;
    private final double velocity;
    private final double leftSideOffset;

    private double leftDesiredPosition = 0;
    private double rightDesiredPosition = 0;

    /**
     * @param climberSubsystem
     * @param velocity velocity in rev/s
     * @param leftSideOffset how many revolutions the left motor should be offset from the right motor
     */
    public OffsetClimbCommand(ClimberSubsystem climberSubsystem, double velocity, double leftSideOffset) {
        this.climberSubsystem = climberSubsystem;
        this.velocity = velocity;
        this.leftSideOffset = leftSideOffset;
    }

    @Override
    public void initialize() {
        rightDesiredPosition = climberSubsystem.getRightMotorPosition();
        leftDesiredPosition = climberSubsystem.getLeftMotorPosition();
    }

    @Override
    public void execute() {

        // Positive error means the left side needs to move up (+) ie. it is currently too low
        double currentOffsetError = leftSideOffset - (climberSubsystem.getLeftMotorPosition() - climberSubsystem.getRightMotorPosition());
        double tolerance = 0.5;

        // If the hooks are in the correct positions relative to each other, we can move both sides
        // If the hooks are not in the correct positions relative to each other, we need to move only one side.
        // The hooks should only move in the commanded direction. So, we need to determine which side will close the error when moved in the given direction.
        if(Math.abs(currentOffsetError) <= tolerance) {
            // Offset is within the tolerance, move both sides
            leftDesiredPosition += velocity * 0.02;
            rightDesiredPosition += velocity * 0.02;
        }else if(currentOffsetError * velocity > 0) {
            // Error and velocity are in the same direction, move left only
            leftDesiredPosition += velocity * 0.02;
        }else {
            // Error and velocity are in the opposite direction, move right only
            rightDesiredPosition += velocity * 0.02;
        }

        climberSubsystem.setLeftMotorPosition(leftDesiredPosition);
        climberSubsystem.setRightMotorPosition(rightDesiredPosition);
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.holdClimberPosition();
    }

}
