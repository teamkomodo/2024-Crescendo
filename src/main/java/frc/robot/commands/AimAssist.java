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
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
    private final LimeLight limelight;

    private boolean continueToServo;
    private double setPoint;
    private DoubleSupplier gyroAngle;
    private double iterationsSinceLostTarget = 0;
    private final double LostTarget_Iterations = 10; //FIXME: don't really trust this num so when testing happens figure it out
    
    
   
    
}
