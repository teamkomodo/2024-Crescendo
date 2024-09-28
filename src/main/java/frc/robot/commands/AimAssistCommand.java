package frc.robot.commands;
 
import static frc.robot.Constants.ANGULAR_VELOCITY_CONSTRAINT;
import static frc.robot.Constants.MAX_ATTAINABLE_VELOCITY;

import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;
//import frc.LimeLightVison.LimeLight; //TODO: prob delete this jum,
import frc.robot.subsystems.DrivetrainSubsystem;



public class AimAssistCommand extends Command{

    //Drivetrain
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    
  
//Dum dum code
    //Limelight stuff
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    private double drive_cmd;
    private double steer_cmd;



    @Override
    public void execute(){
        
        /*Method 2 */
        final var angular_limelight = limelight_aim_proportion();
        
        final var forward_limelight = limelight_range_proportion();

        drivetrainSubsystem.drive(forward_limelight, 0, angular_limelight, true, true);

        System.out.println("Drive command: " + forward_limelight);
        System.out.println("Angular command: " + angular_limelight);

        
        
    }

    @Override
    public void end(boolean interrupted){
        drivetrainSubsystem.stopMotion();
        System.out.println("stopped robor");
    }
    

   
    double limelight_aim_proportion(){

        double aimP = 0.5; //TODO: tune this

        double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * aimP;

        targetingAngularVelocity *= ANGULAR_VELOCITY_CONSTRAINT;

        return targetingAngularVelocity;
    }

    double limelight_range_proportion(){
        double rangeP = 0.1; //TODO: tune this

        double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * rangeP;
        targetingForwardSpeed *= MAX_ATTAINABLE_VELOCITY;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }

    private void Update_Limelight_Tracking(){
        //FIXME:Tune these nums to see how fast we want robor to ber
        final double STEER_K = 0.00;
        final double DRIVE_K = 0;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.0; 
        
        boolean tv = LimelightHelpers.getTV("limelight"); //true if a target is detected
        double tx = LimelightHelpers.getTX("limelight");
        double ty = LimelightHelpers.getTY("limelight");
        double ta = LimelightHelpers.getTA("limelight"); //size of target as percentage
                                

        if(tv == false){
            m_LimelightHasValidTarget = false;
            m_LimelightDriveCommand = 0.0;
            m_LimelightSteerCommand = 0.0;
            return;
        }

        m_LimelightHasValidTarget = true;

        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        if(drive_cmd > MAX_DRIVE){
            drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
        
    }
    
}
