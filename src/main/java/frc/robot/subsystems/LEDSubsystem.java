package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Patterns;

public class LEDSubsystem extends SubsystemBase {
    public static final double IDLE_PATTERN = 0.43; //Sparkle, Color 1 on Color 2
    private static final int LED_PWM_CHANNEL = 0;

    private Spark controller = new Spark(LED_PWM_CHANNEL);

    public void setPattern(double pattern){
        controller.set(pattern);
    }

    public Command setPatternCommand(double pattern) {
        return this.runOnce(() -> setPattern(pattern));
    }

    public Command idlePatternCommand() {
        return setPatternCommand(IDLE_PATTERN);
    }
    
    public Command setSolidColor(int colorId) {
        // Not including black
        return this.runOnce(() -> setPattern(0.57 + (0.02 * (colorId % 21))));
    }

    public Command defaultColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }

    public Command autonomousColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }

    public Command flywheelsRampingColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_RED);
    }

    public Command flywheelsRampedColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }

    public Command intakeColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }

    public Command ampColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }

    public Command climbingColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }

    public Command trapColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }
}
