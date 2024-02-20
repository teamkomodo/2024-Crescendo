package frc.robot.util;

public class PIDGains {
    
    public final double kP;
    public final double kI;
    public final double kD;

    public PIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }
    
}
