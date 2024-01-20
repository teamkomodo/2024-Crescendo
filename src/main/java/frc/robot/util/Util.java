package frc.robot.util;

public class Util {

    public static double joystickCurve(double input) {
        return input;
        // double magnitude = Math.abs(input);
        // if(magnitude < 0.5) {
        //     return Math.signum(input) * (Math.pow(magnitude, 3) + .25 * input);
        // }
        
        // return Math.signum(input) * (-1 * Math.pow(magnitude, 3) + 3 * Math.pow(magnitude, 2) - 1.25 * magnitude + .25);
    }

}
