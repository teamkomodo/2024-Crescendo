/*Turbotake subsystem code
*Intake/Shooter
* 2 Shooter Motors
* 1 indexer motor
*/
package frc.robot.subsystems;

//Libraries
import edu.wpi.first.wpilibj.DigitalInput;
//This library isn't used but I might use it later
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
//Motor Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

//IDs from Constants
import static frc.robot.Constants.BEAM_BREAK_SENSOR_PORT;
import static frc.robot.Constants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_ID1;
import static frc.robot.Constants.SHOOTER_MOTOR_ID2;


public class TurboTakeSubsystem extends SubsystemBase{
     //defines motors
     private CANSparkMax shooterMotor1;
     private CANSparkMax indexerMotor;
     private CANSparkMax shooterMotor2;

     //defines beam break sensor
     private DigitalInput beamBreakSensor;

     //defines PID and its values
     //p i d values are not factual because turbotake isn't built yet
     private SparkPIDController pidController;
     private double p = 1;
     private double i = 0;
     private double d = 0;
     private double maxIAccum = 0;
     
     //init outtake motors and restores factory defaults
     public void TurbotakeSubsystem(){
        //Initialize the motors
        shooterMotor1 = new CANSparkMax(SHOOTER_MOTOR_ID1, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(SHOOTER_MOTOR_ID2, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
        
        //Initialize the beam break sensor
        beamBreakSensor = new DigitalInput(BEAM_BREAK_SENSOR_PORT);
        
        //restores controller parameters to factory defaults for motors
        indexerMotor.restoreFactoryDefaults();
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor1.restoreFactoryDefaults();

        pidController = indexerMotor.getPIDController();
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setIMaxAccum(maxIAccum, 0);
     }

     //Turns on shooter motor at 12 volts * shooterPercent
     public  void SetShooterSpeed(double shooterPercent){
        shooterMotor1.set(shooterPercent);
        //motor2 needs to spin outwards with some extra SPIN
        shooterMotor2.set(-shooterPercent * 1.75);
     }

     //Turns on indexer motor at 12 volts * indexerPercent
     public  void SetIndexerSpeed(double indexerPercent){
        indexerMotor.set(indexerPercent);
     }
     //returns false if something breaks the beam
     public boolean pieceDetected(){
        return !beamBreakSensor.get();
     }

     
     public void setMotorDutyCycle(double dutyCycle){
         pidController.setReference(dutyCycle, ControlType.kDutyCycle);
     }


}