// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

    public static final double MAX_ATTAINABLE_VELOCITY = 3.8;

    public static final double LINEAR_VELOCITY_CONSTRAINT = MAX_ATTAINABLE_VELOCITY;
    public static final double LINEAR_ACCEL_CONSTRAINT = 12.0;

    public static final double ANGULAR_VELOCITY_CONSTRAINT = (LINEAR_VELOCITY_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH);
    public static final double ANGULAR_ACCEL_CONSTRAINT = (LINEAR_ACCEL_CONSTRAINT * Math.PI) / (DRIVETRAIN_WIDTH * DRIVETRAIN_WIDTH + DRIVETRAIN_LENGTH * DRIVETRAIN_LENGTH);

    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
        new PIDConstants(1.0, 0, 0),
        new PIDConstants(0, 0, 0),
        MAX_ATTAINABLE_VELOCITY,
        Math.sqrt(DRIVETRAIN_LENGTH*DRIVETRAIN_LENGTH + DRIVETRAIN_WIDTH*DRIVETRAIN_WIDTH)/2,
        new ReplanningConfig()
    );

    // FRC Field
    public static final double FIELD_WIDTH = 821; //cm approxiamation Field Length is 26ft. 11 1/8 in wide
    public static final double FIELD_LENGTH = 1654;
}
