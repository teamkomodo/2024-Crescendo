package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class LEDSubsystem extends SubsystemBase {
    public static final double IDLE_PATTERN = 0.43; //Sparkle, Color 1 on Color 2
    public static final double CONE_SIGNAL_PATTERN = 0.65; //Solid Color: Orange
    public static final double CUBE_SIGNAL_PATTERN = 0.91; //Solid Color: Violet

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

    public Command coneSignalCommand() {
        return setPatternCommand(CONE_SIGNAL_PATTERN);
    }

    public Command cubeSignalCommand() {
        return setPatternCommand(CUBE_SIGNAL_PATTERN);
    }
}
