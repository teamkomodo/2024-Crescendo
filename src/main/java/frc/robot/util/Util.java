package frc.robot.util;

public class Util {

    public static double translationCurve(double input) {
        return Math.pow(Math.abs(input), 1.5) * Math.signum(input);
        // double magnitude = Math.abs(input);
        // if(magnitude < 0.5) {
        //     return Math.signum(input) * (Math.pow(magnitude, 3) + .25 * input);
        // }
        
        // return Math.signum(input) * (-1 * Math.pow(magnitude, 3) + 3 * Math.pow(magnitude, 2) - 1.25 * magnitude + .25);
    }

    public static double steerCurve(double input) {
        return Math.pow(input, 3);
    }

}
