package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.LimelightHelpers;


//do i know what this even means? no i do not
public class AutoRangedShooting {
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();

    double limelight_aim_proportional(){
        
        //aim proportional
        //TODO: Finetune num for aim proportional
        double kAP = .035; 

        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kAP;

        targetingAngularVelocity *= drivetrainSubsystem.kMaxAngularSpeed;
        
        targetingAngularVelocity *= -1.0;

        return targetingAngularVelocity;
    }
    

    double limelight_range_proportional(){
        //range proportional
        //TODO: Finetune the num for range proportional
        double kRP = .1;
        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kRP;
        targetingForwardSpeed *= drivetrainSubsystem.kMaxSpeed;
        targetingForwardSpeed *= -1.0;

        return targetingForwardSpeed;
    }


     
}
