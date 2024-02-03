// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

// Controls
    public static final int DRIVER_XBOX_PORT = 0;

// Turbotake
    //Motors
    public static final int SHOOTER_MOTOR_ID1 = 10;
    public static final int SHOOTER_MOTOR_ID2 = 11;
    public static final int INDEXER_MOTOR_ID = 32;
    //Beam Break Sensor
    public static final int BEAM_BREAK_SENSOR_PORT = 1;
    //Constants
    public static final double INDEXER_SPEED = 0.7;
    public static final double SHOOTER_SPEED = 1;
// Drivetrain
    public static final boolean FIELD_RELATIVE_DRIVE = true;
    public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
    public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.2;

    public static final double DRIVETRAIN_WIDTH = 0.5271; // Distance between center of left and right swerve wheels in meters
    public static final double DRIVETRAIN_LENGTH = 0.5271; // Distance between center of front and back swerve wheels in meters

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 16;
    public static final int FRONT_LEFT_STEER_MOTOR_ID = 17;
    public static final int FRONT_LEFT_STEER_ENCODER_ID = 23;
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(24.697);

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 12;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 13;
    public static final int FRONT_RIGHT_STEER_ENCODER_ID = 21;
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(-72.510);

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 14;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 15;
    public static final int BACK_LEFT_STEER_ENCODER_ID = 22;
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(174.841);

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 10;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 11;
    public static final int BACK_RIGHT_STEER_ENCODER_ID = 20;
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(57.920);

    public static final double WHEEL_DIAMETER = 0.1016;

    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0); // reduction * motor rpm = wheel rpm
    public static final double STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    public static final double NEO_BRUSHLESS_FREE_RPM = 5676.0;
    public static final double NEO_KV = 12.0/NEO_BRUSHLESS_FREE_RPM; // V/RPM

    public static final double MAX_LINEAR_VELOCITY = 3.6;
    public static final double MAX_LINEAR_ACCEL = 6.0;

    public static final double MAX_ANGULAR_VELOCITY = 4.0 * Math.PI;
    public static final double MAX_ANGULAR_ACCEL = 4.0 * Math.PI;
}
