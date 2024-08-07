package frc.robot.commands;
 
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.LimeLightVison.LimeLight;

public class AimAssist extends Command{
    //TODO: add subsystems when i can get them to follow this
    private final DrivetrainSubsystem drivetrainSubsystem;

    private final LimeLight m_LimeLight;

    private boolean m_continueToServo; //not quite sure if ill need this one
    private double m_setPoint;
    private DoubleSupplier m_gyroAngle;
    private double m_iterationsSinceLostTarget = 0;
    private final double m_LostTarget_Iterations = 10; //FIXME: don't really trust this num so when testing happens figure it out
    
    

    //addRequirements() declares 
    public AimAssist(DrivetrainSubsystem subsystem, DoubleSupplier gyroAngle, LimeLight limelight){
        drivetrainSubsystem = subsystem;
        addRequirements(drivetrainSubsystem);
        m_gyroAngle = gyroAngle;
        m_LimeLight = limelight;
    }


    //Calls when command is intitally scheduled
    @Override
    public void initialize(){
        m_LimeLight.setPipeline(5);
        m_iterationsSinceLostTarget = m_LostTarget_Iterations;
    }

    @Override
    public void execute(){

        double kp = .9; //FIXME: dont trust this num when test fix

        if(m_LimeLight.getIsTargetFound()){
            //TODO: add setpoint stuff to drivetrain that makes it do the thing

        } else{
            //mhm i for sure know what this code is gonna mean when i have to hack it to work with our stuff
        }
    }
    
}
