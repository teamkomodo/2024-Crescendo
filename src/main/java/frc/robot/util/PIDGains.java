package frc.robot.util;

public class PIDGains {
    
    public final double p;
    public final double i;
    public final double d;
    public final double maxIAccum;
    public final double iZone;
    public final double FF;
    public final double minOutput;
    public final double maxOutput;
    public int numArgs;

    public PIDGains(double p, double i, double d) {
        this(p, i, d, 0);
        this.numArgs = 3;
    }

    public PIDGains(double p, double i, double d, double maxIAccum) {
        this(p, i, d, maxIAccum, 0, 0, 0, 0);
        this.numArgs = 4;
    }

    public PIDGains(double p, double i, double d, double maxIAccum, double iZone, double FF, double minOutput, double maxOutput) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.maxIAccum = maxIAccum;
        this.iZone = iZone;
        this.FF = FF;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.numArgs = 8;
    }
    
}
