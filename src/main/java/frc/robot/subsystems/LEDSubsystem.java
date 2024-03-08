package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.BlinkinPattern;

import static frc.robot.Constants.*;

public class LEDSubsystem extends SubsystemBase {
    public static final double IDLE_PATTERN = 0.43; //Sparkle, Color 1 on Color 2

    private Spark controller = new Spark(LED_CHANNEL);

    public void setPattern(double pattern){
        controller.set(pattern);
    }

    public Command setPatternCommand(double pattern) {
        return Commands.runOnce(() -> setPattern(pattern));
    }

    public Command setSolidColorCommand(int colorId) {
        // Not including black
        return Commands.runOnce(() -> setPattern(0.57 + (0.02 * (colorId % 21))));
    }

    public Command idlePatternCommand() {
        return setPatternCommand(IDLE_PATTERN);
    }

    public Command defaultColorCommand() {
        return setPatternCommand(BlinkinPattern.COLOR_1_AND_2_PATTERN_SPARKLE_COLOR_1_ON_COLOR_2);
    }

    public Command autonomousColorCommand() {
        return setPatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_COLOR_WAVES_OCEAN_PALETTE);
    }

    public Command flywheelsRampingColorCommand() {
        return setPatternCommand(BlinkinPattern.COLOR_1_PATTERN_STROBE);
    }

    public Command flywheelsRampedColorCommand() {
        return setPatternCommand(BlinkinPattern.SOLID_COLORS_BLUE);
    }

    public Command intakeColorCommand() {
        return setPatternCommand(BlinkinPattern.SOLID_COLORS_RED);
    }

    public Command pieceLoadedColorCommand() {
        return setPatternCommand(BlinkinPattern.SOLID_COLORS_GREEN);
    }

    public Command alignedToFieldElementColorCommand() {
        return setPatternCommand(BlinkinPattern.FIXED_PALETTE_PATTERN_RAINBOW_LAVA_PALETTE);
    }
}
