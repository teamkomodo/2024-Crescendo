// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final boolean TUNING_MODE = false;

// Controls
    public static final double XBOX_DEADBAND = 0.06;
    public static final int DRIVER_XBOX_PORT = 0;
    public static final int OPERATOR_XBOX_PORT = 1;

// Turbotake
    //Motors
    public static final int LEFT_SHOOTER_MOTOR_ID = 36;
    public static final int RIGHT_SHOOTER_MOTOR_ID = 37;
    public static final int INDEXER_MOTOR_ID = 35;
    //Beam Break Sensor
    public static final int INTAKE_BEAM_BREAK_PORT = 3;

    //Constants
    public static final double INDEXER_SPEED = 0.7;
    
    public static final double SHOOTER_SPEED = 2500;
    public static final double SHOOTER_MAX_VELOCITY = 2900;

    //duty cycle
    public static final double AMP_SPEED = 0.75;
    //rpm
    public static final double SPEAKER_SPEED = 2500;
    public static final double SPIN_RATIO = 0.5;

    public static final double LINEAR_SLOW_MODE_MODIFIER = 0.5;
    public static final double ANGULAR_SLOW_MODE_MODIFIER = 0.3;
    public static final double DRIVETRAIN_WIDTH = 0.5271; // Distance between center of left and right swerve wheels in meters
    public static final double DRIVETRAIN_LENGTH = 0.5271; // Distance between center of front and back swerve wheels in meters

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 16;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 17;
    public static final int BACK_RIGHT_STEER_ENCODER_ID = 23;
    public static final double BACK_RIGHT_STEER_OFFSET = -Math.toRadians(24.697 + 180);

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 12;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 13;
    public static final int BACK_LEFT_STEER_ENCODER_ID = 21;
    public static final double BACK_LEFT_STEER_OFFSET = -Math.toRadians(-72.510 + 180);

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 14;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 15;
    public static final int FRONT_RIGHT_STEER_ENCODER_ID = 22;
    public static final double FRONT_RIGHT_STEER_OFFSET = -Math.toRadians(174.841 + 180);

    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 10;
    public static final int FRONT_LEFT_STEER_MOTOR_ID = 11;
    public static final int FONT_LEFT_STEER_ENCODER_ID = 20;
    public static final double FRONT_LEFT_STEER_OFFSET = -Math.toRadians(57.920 + 180);

    public static final double WHEEL_DIAMETER = 0.1016;

    /**
     * motor rotations -> wheel rotations
     */
    public static final double DRIVE_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0);

    /**
     * motor rotations -> module rotations
     */
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

    public static final BooleanSupplier ON_RED_ALLIANCE = () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if(alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            };

// Joint/Elevator
    public static final int JOINT_MOTOR_ID = 32;
    public static final int JOINT_SECOND_MOTOR_ID = 33;
    public static final int JOINT_MIDDLE_ZERO_SWITCH_CHANNEL = 1;
    public static final int JOINT_BOTTOM_ZERO_SWITCH_CHANNEL = 0;

    public static final double JOINT_MIN_POSITION = 2; // Code stop
    public static final double JOINT_MAX_POSITION = 47; // Code stop

    public static final double JOINT_BOTTOM_SWITCH_POSITION = 0;
    public static final double JOINT_STARTING_POSITION = 4.1;
    public static final double JOINT_LEVEL_OFFSET = -4;

    /**
     * motor rotations -> mechanism rotations
     */
    public static final double JOINT_REDUTION = (1.0 / 100.0) * (18.0 / 30.0);
    public static final double TURBOTAKE_JOINT_RADIAN_OFFSET = 1.91986; // 110 degrees
    public static final double JOINT_AVERAGE_SHOOT_HEIGHT = 0;
    
    public static final double JOINT_STOW_POSITION = 10;
    public static final double JOINT_AMP_POSITION = 30;
    public static final double JOINT_SPEAKER_POSITION = 16;
    public static final double JOINT_PRE_INTAKE_POSITION = 4;
    public static final double JOINT_TRAP_POSITION = 45;
    public static final double JOINT_INTAKE_POSITION = 1.5;
    public static final double JOINT_CLIMB_POSITION = 1.5;

    /**
     * motor rotations -> joint radians
     */
    public static final double JOINT_ANGLE_CONVERSION_FACTOR = JOINT_REDUTION * 2.0 * Math.PI;
    
    public static final double JOINT_MOVING_VELOCITY = 80;

    public static final int ELEVATOR_MOTOR_ID = 34;
    public static final int ELEVATOR_ZERO_SWITCH_CHANNEL = 2;
    public static final double ELEVATOR_SLOW_MODE_MULTIPLIER = 0.5;

    // Position in rotations of the motor shaft before gearbox
    public static final double ELEVATOR_MIN_POSITION = 0; // Code stop
    public static final double ELEVATOR_MAX_POSITION = 64.5; // Code stop

    public static final double ELEVATOR_STOW_POSITION = 0;
    public static final double ELEVATOR_AMP_POSITION = 55;
    public static final double ELEVATOR_SPEAKER_POSITION = 0;
    public static final double ELEVATOR_TRAP_POSITION = 0;
    public static final double ELEVATOR_INTAKE_POSITION = 63;
    public static final double ELEVATOR_CLIMB_POSITION = 63;

    // FRC Field
    public static final double FIELD_WIDTH = 8.21; // m approxiamation: Field Length is 26ft. 11 1/8 in wide
    public static final double FIELD_LENGTH = 16.54;

    // Climber
    public static final int CLIMBER_MOTOR_RIGHT_ID = 39;
    public static final int CLIMBER_MOTOR_LEFT_ID = 38;

    public static final int CLIMBER_MOTOR_RIGHT_BEAM_BREAK_ID = 4;
    public static final int CLIMBER_MOTOR_LEFT_BEAM_BREAK_ID = 5;

    public static final double CLIMBER_MIN_POSITION = 0;
    public static final double CLIMBER_MAX_POSITION = 66;

    public static final double CLIMBER_PRE_CLIMB_POSITION = 20;
    public static final double CLIMBER_POST_CLIMB_POSITION = -40;

    public static final double CLIMBER_EXTEND_VELOCITY = 80;
    public static final double CLIMBER_ASCEND_VELOCITY = 80;

    public static final int A_FRAME_LED_CHANNEL = 0;
    public static final int TURBOTAKE_LED_CHANNEL = 1;

}
