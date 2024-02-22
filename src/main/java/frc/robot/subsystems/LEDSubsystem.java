package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Patterns;

import static frc.robot.Constants.*;

public class LEDSubsystem extends SubsystemBase {
    public static final double IDLE_PATTERN = 0.43; //Sparkle, Color 1 on Color 2

    private Spark controller = new Spark(LED_CHANNEL);

    public void setPattern(double pattern){
        controller.set(pattern);
    }

    public Command setPatternCommand(double pattern) {
        return this.runOnce(() -> setPattern(pattern));
    }

    public Command setSolidColorCommand(int colorId) {
        // Not including black
        return this.runOnce(() -> setPattern(0.57 + (0.02 * (colorId % 21))));
    }

    public Command idlePatternCommand() {
        return setPatternCommand(IDLE_PATTERN);
    }

    public Command defaultColor() {
        return setPatternCommand(Patterns.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_1_ON_COLOR_2);
    }

    public Command autonomousColor() {
        return setPatternCommand(Patterns.FIXED_PALETTE_PATTERN_COLOR_WAVES_OCEAN_PALETTE);
    }

    public Command flywheelsRampingColor() {
        return setPatternCommand(Patterns.COLOR_1_PATTERN_STROBE);
    }

    public Command flywheelsRampedColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_BLUE);
    }

    public Command intakeColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_RED);
    }

    public Command pieceLoadedColor() {
        return setPatternCommand(Patterns.SOLID_COLORS_GREEN);
    }

    public Command alignedToFieldElementColor() {
        return setPatternCommand(Patterns.FIXED_PALETTE_PATTERN_RAINBOW_LAVA_PALETTE);
    }
}
