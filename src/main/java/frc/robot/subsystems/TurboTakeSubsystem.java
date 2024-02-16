/*Turbotake subsystem code
*Intake/Shooter
* 2 Shooter Motors
* 1 indexer motor
*/
package frc.robot.subsystems;

//import edu.wpi.first.math.util.Units;
//Libraries
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;



import com.revrobotics.SparkPIDController;
//Motor Libraries
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.units.Units;

//IDs from Constants
import static frc.robot.Constants.BEAM_BREAK_SENSOR_PORT;
import static frc.robot.Constants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_1_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_2_ID;


public class TurboTakeSubsystem extends SubsystemBase{
    //defines motors
    private final CANSparkMax shooterMotor1;
    private final CANSparkMax shooterMotor2;
    private final CANSparkMax indexerMotor;
    
    //Encoders
    private final RelativeEncoder shooter1Encoder;
    private final RelativeEncoder shooter2Encoder;
    private final RelativeEncoder indexerEncoder;
    
    //PID Controllers
    private final SparkPIDController indexerPidController;
    private final SparkPIDController shooter1PidController;
    private final SparkPIDController shooter2PidController;
    //defines beam break sensor
    private final DigitalInput beamBreakSensor;
    
    
    
    //shuffleboard
    public ShuffleboardTab shuffleboardTab;

    
    
    //PID values for indexer
    private double indexerP, indexerI, indexerD, indexerIAccumulator, 
    indexerFF, indexerMinOutput, indexerMaxOutput;
    //PID values for shooterMotor1
    private double shooterP, shooterI, shooterD, shooterIAccumulator, 
    shooterFF, shooterMinOutput, shooterMaxOutput;
    
    public final SysIdRoutine indexerRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            (voltage) -> setIndexerVelocity(voltage.in(Units.Volts)),
            null,
                this
        ));

    public final SysIdRoutine shooterRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
            (voltage) -> setShooterVelocity(voltage.in(Units.Volts)),
            null,
                this
        ));
    
    //init outtake motors and restores factory defaults
    public TurboTakeSubsystem(){
        
        
        // PID coefficients for indexer
        indexerP = 1;
        indexerI = 0;
        indexerD = 0;
        indexerIAccumulator = 0;
        indexerFF = 0;
        indexerMinOutput = -1;
        indexerMaxOutput = 1;
        
        // PID coefficients for shooter motors
        shooterP = 3.78e-04;
        shooterI = 0;
        shooterD = 0;
        shooterIAccumulator = 0;
        shooterFF = 0/*0.0023097*/;
        shooterMinOutput = -1;
        shooterMaxOutput = 1;
        
        
        //Initialize the motors
        shooterMotor1 = new CANSparkMax(SHOOTER_MOTOR_1_ID, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(SHOOTER_MOTOR_2_ID, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
        
        shooterMotor1.setInverted(false);
        shooterMotor2.setInverted(true);
        indexerMotor.setInverted(true);
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
        
    }
  
    //returns false if something breaks the beam
    public boolean pieceDetected(){
        return !beamBreakSensor.get();
    }
    
    // commands the shooter to a target velocity
    public void setShooterVelocity(double velocity){
        shooter1PidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        shooter2PidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
    
    // commands the indexer to a target velocity
    public void setIndexerVelocity(double velocity){
        indexerPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
    
    
    // sets the duty cycle (percent output) of the indexer motor
    public void setIndexerPercent(double percent){
        indexerMotor.set(-percent);
    }
    
    // set the duty cycle (percent output) of the shooter motors
    public void setShooterPercent(double percent){
        shooterMotor1.set(-percent);
        shooterMotor2.set(percent);
    }

}