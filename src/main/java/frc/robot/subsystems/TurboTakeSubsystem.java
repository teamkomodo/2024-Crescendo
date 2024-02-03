/*Turbotake subsystem code
*Intake/Shooter
* 2 Shooter Motors
* 1 indexer motor
*/
package frc.robot.subsystems;

//Libraries
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.SparkPIDController;
//Motor Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

//IDs from Constants
import static frc.robot.Constants.BEAM_BREAK_SENSOR_PORT;
import static frc.robot.Constants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_ID1;
import static frc.robot.Constants.SHOOTER_MOTOR_ID2;


public class TurboTakeSubsystem extends SubsystemBase{
     //defines motors
     private CANSparkMax shooterMotor1;
     private CANSparkMax shooterMotor2;
     private CANSparkMax indexerMotor;

     //Encoders
     private RelativeEncoder shooter1Encoder;
     private RelativeEncoder shooter2Encoder;
     private RelativeEncoder indexerEncoder;
    
     //PID Controllers
     private SparkPIDController indexerPidController;
     private SparkPIDController shooter1PidController;
     private SparkPIDController shooter2PidController;
     //defines beam break sensor
     private DigitalInput beamBreakSensor;

   

     //shuffleboard
     public ShuffleboardTab shuffleboardTab;
     //defines PID and its values
     

     //PID values for indexer
     private double indexerP, indexerI, indexerD, indexerIAccumulator, 
     indexerFF, indexerMinOutput, indexerMaxOutput;
     //PID values for shooterMotor1
     private double shooterP, shooterI, shooterD, shooterIAccumulator, 
     shooterFF, shooterMinOutput, shooterMaxOutput;


     
     //init outtake motors and restores factory defaults
     public void TurbotakeSubsystem(){


      // PID coefficients for indexer
      indexerP = 1;
      indexerI = 0;
      indexerD = 0;
      indexerIAccumulator = 0;
      indexerFF = 0;
      indexerMinOutput = 0;
      indexerMaxOutput = 0;

      // PID coefficients for shooter motors
      shooterP = 1;
      shooterI = 0;
      shooterD = 0;
      shooterIAccumulator = 0;
      shooterFF = 0;
      shooterMinOutput = 0;
      shooterMaxOutput = 0;


      //Initialize the motors
      shooterMotor1 = new CANSparkMax(SHOOTER_MOTOR_ID1, MotorType.kBrushless);
      shooterMotor2 = new CANSparkMax(SHOOTER_MOTOR_ID2, MotorType.kBrushless);
      indexerMotor = new CANSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

      shooterMotor2.setInverted(true);
        
      //Initialize the beam break sensor
      beamBreakSensor = new DigitalInput(BEAM_BREAK_SENSOR_PORT);
        
      //Initializes encoders
      shooter1Encoder = shooterMotor1.getEncoder();
      shooter2Encoder = shooterMotor2.getEncoder();
      indexerEncoder = indexerMotor.getEncoder();

      //sets encoder positions to 0
      shooter1Encoder.setPosition(0);
      shooter2Encoder.setPosition(0);
      indexerEncoder.setPosition(0);

      //restores controller parameters to factory defaults for motors
      indexerMotor.restoreFactoryDefaults();
      shooterMotor2.restoreFactoryDefaults();
      shooterMotor1.restoreFactoryDefaults();
        
        
      //initializes indexer PID controller to the PID controller in indexerMotor
      indexerPidController = indexerMotor.getPIDController();
      //sets PID values
      indexerPidController.setP(indexerP);
      indexerPidController.setI(indexerI);
      indexerPidController.setD(indexerD);
      indexerPidController.setIAccum(indexerIAccumulator);
      indexerPidController.setFF(indexerFF);
      indexerPidController.setOutputRange(indexerMinOutput, indexerMaxOutput);

       //initializes shooter1 and shooter2 PID controllers to the PID controllers in each motor
      shooter1PidController = shooterMotor1.getPIDController();
      shooter2PidController = shooterMotor2.getPIDController();

      //sets PID values for shooter1
      shooter1PidController.setP(shooterP);
      shooter1PidController.setP(shooterI);
      shooter1PidController.setP(shooterD);
      shooter1PidController.setIAccum(shooterIAccumulator);
      shooter1PidController.setFF(shooterFF);
      shooter1PidController.setOutputRange(shooterMinOutput, shooterMaxOutput);

         

      //sets PID values for shooter2
      shooter2PidController.setP(shooterP);
      shooter2PidController.setP(shooterI);
      shooter2PidController.setP(shooterD);
      shooter2PidController.setIAccum(shooterIAccumulator);
      shooter2PidController.setFF(shooterFF);
      shooter2PidController.setOutputRange(shooterMinOutput, shooterMaxOutput);

      //Smart Dashboard Displays the PID coefficients for indexer
      SmartDashboard.putNumber("Indexer P Gain", indexerP);
      SmartDashboard.putNumber("Indexer I Gain", indexerI);
      SmartDashboard.putNumber("Indexer D Gain", indexerD);
      SmartDashboard.putNumber("Indexer I Accumulation", indexerIAccumulator);
      SmartDashboard.putNumber("Indexer Feed Forward", indexerFF);
      SmartDashboard.putNumber("Indexer Max Output", indexerMaxOutput);
      SmartDashboard.putNumber("Indexer Min Output", indexerMinOutput);


      //Smart dashboard displays PID coefficients for shooters
      SmartDashboard.putNumber("Shooter P Gain", indexerP);
      SmartDashboard.putNumber("Shooter I Gain", indexerI);
      SmartDashboard.putNumber("Shooter D Gain", indexerD);
      SmartDashboard.putNumber("Shooter I Accumulation", indexerIAccumulator);
      SmartDashboard.putNumber("Shooter Feed Forward", indexerFF);
      SmartDashboard.putNumber("Shooter Max Output", indexerMaxOutput);
      SmartDashboard.putNumber("Shooter Min Output", indexerMinOutput);
     
    }

     public void updateShuffleboard(){

      //Indexer real PID coefficients
      double indexerRealP = SmartDashboard.getNumber("Indexer P Gain", 0);
      double indexerRealI = SmartDashboard.getNumber("Indexer I Gain", 0);
      double indexerRealD = SmartDashboard.getNumber("Indexer D Gain", 0);
      double indexerRealIAcumulator = SmartDashboard.getNumber("Indexer I Acummulation", 0);
      double indexerRealFF = SmartDashboard.getNumber("Indexer Feed Forward", 0);
      double indexerRealMax = SmartDashboard.getNumber("Indexer Max output", 0);
      double indexerRealMin = SmartDashboard.getNumber("Indexer Min output", 0);
   
   
      //shooter real PID coefficients
      double shooterRealP = SmartDashboard.getNumber("Shooter P Gain", 0);
      double shooterRealI = SmartDashboard.getNumber("Shooter I Gain", 0);
      double shooterRealD = SmartDashboard.getNumber("Shooter D Gain", 0);
      double shooterRealIAcumulator = SmartDashboard.getNumber("Shooter I Acummulation", 0);
      double shooterRealFF = SmartDashboard.getNumber("Shooter Feed Forward", 0);
      double shooterRealMax = SmartDashboard.getNumber("Shooter Max output", 0);
      double shooterRealMin = SmartDashboard.getNumber("Shooter Min output", 0);


      //If the values of real are different that the ones we set then we make the ones we set equal to the real values
      if((indexerRealP != indexerP)) { indexerPidController.setP(indexerRealP); indexerP = indexerRealP;}
      if((indexerRealI != indexerI)) { indexerPidController.setI(indexerRealI); indexerI = indexerRealI;}
      if((indexerRealD != indexerD)) { indexerPidController.setD(indexerRealD); indexerD = indexerRealD;}
      if((indexerRealIAcumulator != indexerIAccumulator)) { indexerPidController.setIAccum(indexerRealIAcumulator); indexerIAccumulator = indexerRealIAcumulator;}
      if((indexerRealFF != indexerFF)) { indexerPidController.setFF(indexerRealFF); indexerFF = indexerRealFF;}
      
      if((indexerRealMax != indexerMaxOutput) || (indexerRealMin != indexerMinOutput)){
        indexerPidController.setOutputRange(indexerRealMin, indexerRealMin);
        indexerMinOutput = indexerRealMin;
        indexerMaxOutput = indexerRealMax;
      }


      //
      if((shooterRealP != shooterP)) { shooter1PidController.setP(shooterRealP); shooterP = shooterRealP;}
      if((shooterRealI != shooterI)) { shooter1PidController.setI(shooterRealI); shooterI = shooterRealI;}
      if((shooterRealD != shooterD)) { shooter1PidController.setD(shooterRealD); shooterD = shooterRealD;}
      if((shooterRealIAcumulator != indexerIAccumulator)) { shooter1PidController.setIAccum(shooterRealIAcumulator); shooterIAccumulator = shooterRealIAcumulator;}
      if((shooterRealFF != shooterFF)) { shooter1PidController.setFF(shooterRealFF); shooterFF = shooterRealFF;}
      
      if((shooterRealMax != shooterMaxOutput) || (shooterRealMin != shooterMinOutput)){
        shooter1PidController.setOutputRange(shooterRealMin, shooterRealMin);
        shooterMinOutput = shooterRealMin;
        shooterMaxOutput = shooterRealMax;
      }


      if((shooterRealP != shooterP)) { shooter2PidController.setP(shooterRealP); shooterP = shooterRealP;}
      if((shooterRealI != shooterI)) { shooter2PidController.setI(shooterRealI); shooterI = shooterRealI;}
      if((shooterRealD != shooterD)) { shooter2PidController.setD(shooterRealD); shooterD = shooterRealD;}
      if((shooterRealIAcumulator != indexerIAccumulator)) { shooter2PidController.setIAccum(shooterRealIAcumulator); shooterIAccumulator = shooterRealIAcumulator;}
      if((shooterRealFF != shooterFF)) { shooter2PidController.setFF(shooterRealFF); shooterFF = shooterRealFF;}
      
      if((shooterRealMax != shooterMaxOutput) || (shooterRealMin != shooterMinOutput)){
        shooter2PidController.setOutputRange(shooterRealMin, shooterRealMin);
        shooterMinOutput = shooterRealMin;
        shooterMaxOutput = shooterRealMax;
      }
     }
     
     //returns false if something breaks the beam
     public boolean pieceDetected(){
        return !beamBreakSensor.get();
     }

     //sets reference for indexer motor so they can start moving
     public void SetShooterSpeed(double shooterPercent){
      shooter1PidController.setReference(shooterPercent, CANSparkMax.ControlType.kVelocity);
      shooter2PidController.setReference(shooterPercent, CANSparkMax.ControlType.kVelocity);
     }

     //sets reference for shooter motors so they can move
     public void SetIndexerSpeed(double indexerPercent){
      indexerPidController.setReference(indexerPercent, CANSparkMax.ControlType.kVelocity);
     }


     //Collects and displays turbotake data
     public ShuffleboardTab getTab(){
         return shuffleboardTab;
     }


     
}