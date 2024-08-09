package frc.robot.commands;
 
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.TeleopStateMachine;
import frc.robot.TeleopStateMachine.State;
import frc.robot.subsystems.DrivetrainSubsystem;

import frc.LimeLightVison.LimeLight;

public class AimAssist extends Command{
    //TODO: add subsystems when i can get them to follow this
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final TeleopStateMachine teleopStateMachine;


    private final LimeLight m_LimeLight;

    private boolean m_continueToServo; //not quite sure if ill need this one
    private double m_setPoint;
    private DoubleSupplier m_gyroAngle;
    private double m_iterationsSinceLostTarget = 0;
    private final double m_LostTarget_Iterations = 10; //FIXME: don't really trust this num so when testing happens figure it out
    
    

    //addRequirements() declares 
    public AimAssist(DrivetrainSubsystem subsystem,TeleopStateMachine stateMachine,DoubleSupplier gyroAngle, LimeLight limelight){
        drivetrainSubsystem = subsystem;
        teleopStateMachine = stateMachine;
        addRequirements(drivetrainSubsystem);
        m_gyroAngle = gyroAngle;
        m_LimeLight = limelight;
    }


    //Calls when command is intitally scheduled
    @Override
    public void initialize(){
        m_LimeLight.setPipeline(5);
        m_iterationsSinceLostTarget = m_LostTarget_Iterations;
        teleopStateMachine.shootSpeakerFarCommand();
    }

    @Override
    public void execute(){

        double kp = .9; //FIXME: dont trust this num when test fix

        if(m_LimeLight.getIsTargetFound())
        {
            m_setPoint = get_Jog_Setpoint(m_LimeLight.getdegRotationToTarget() * kp);
            //TODO: add the thing that uses setpoint for drivetrain

            m_iterationsSinceLostTarget = 0;

        } else{
            m_iterationsSinceLostTarget++;
            //Delay before return to gyro search
            if(m_iterationsSinceLostTarget >= m_LostTarget_Iterations){
                double m_setPoint = MathUtil.inputModulus(m_gyroAngle.getAsDouble(), -70, 290);
                //look at the todo above same code
            }
        }
    }

    @Override
    public void end(boolean interuppted){
        m_LimeLight.setPipeline(0);
        //statemachine stuff
        teleopStateMachine.stateSwitched = true;
        //have statemachine go to drive with piece
        teleopStateMachine.currentState = State.DRIVE_WITH_PIECE;

    }


    @Override
    public boolean isFinished(){
        return false;
    }


    private double get_Jog_Setpoint(double setpoint){
        
        //TODO:Write a get position function to drivetrain to do the logic from the repo im basing code off of

        return 0;
    }

    private double deadband(double setpoint){
        double deadband = .1;
        if(Math.abs(setpoint) < deadband){
            return 0.0;
        } else{
            return Math.copySign(setpoint, setpoint);
        }
    }
    
}
