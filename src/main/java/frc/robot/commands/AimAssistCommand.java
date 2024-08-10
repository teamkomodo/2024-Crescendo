package frc.robot.commands;
 
import java.lang.reflect.Array;
import java.util.function.DoubleSupplier;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.LimeLightVison.LimeLight; //TODO: prob delete this jum,
import frc.robot.subsystems.DrivetrainSubsystem;



public class AimAssistCommand extends Command{

    //Drivetrain
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    
  

    //Limelight stuff
    private boolean m_LimelightHasValidTarget = false;
    private double m_LimelightDriveCommand = 0.0;
    private double m_LimelightSteerCommand = 0.0;

    private double drive_cmd;
    private double steer_cmd;



    @Override
    public void execute(){
        Update_Limelight_Tracking();

        
        if(m_LimelightHasValidTarget){
            
            drivetrainSubsystem.drive(drive_cmd, steer_cmd, 0, false, true);
        }
    }
    

    private void Update_Limelight_Tracking(){
        //FIXME:Tune these nums to see how fast we want robor to ber
        final double STEER_K = 0.00;
        final double DRIVE_K = 0;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.0; 

        
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);//checks if detects a target
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);//x coordinate of center of target in degrees
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);//y coordinate of center of target in degrees
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);//size of target as percentage
                                

        if(tv < 1.0){
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
