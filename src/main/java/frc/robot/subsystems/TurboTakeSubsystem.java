package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;

import static frc.robot.Constants.*;

public class TurbotakeSubsystem extends SubsystemBase{
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
    
    
    
    //Telemetry

    //motor telemetry
    private final DoublePublisher shooter1RPMPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("shooter1 RPM").publish();
    private final DoublePublisher shooter2RPMPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("shooter2 RPM").publish();
    private final DoublePublisher indexerRPMPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("indexer RPM").publish();

    //beam break sensor telemetry
    private final BooleanPublisher beamBreakBooleanPublisher = NetworkTableInstance.getDefault().getBooleanTopic("beam break").publish();

    private boolean hasPiece = false;

    
    //PID values for indexer
    private double indexerP, indexerI, indexerD, indexerIAccumulator, indexerFF, indexerMinOutput, indexerMaxOutput;
    //PID values for shooter motors
    private double shooterP, shooterI, shooterD, shooterIAccumulator, shooterFF, shooterMinOutput, shooterMaxOutput;
    
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
    

    public static enum TurbotakeState {
        IDLE,
        INTAKE,
        AMP_SHOOT,
        SPEAKER_SHOOT,
        TRAP_SHOOT
    }

    //current turbotake state
    private TurbotakeState currentState = TurbotakeState.IDLE;

    //true when state exit condition is met and runs next state
    private boolean stateSwitched = false;

    //private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    //init outtake motors and restores factory defaults
    public TurbotakeSubsystem(){
        
        
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

    public void setShooterPercent(double percent){
        setShootPercent(percent, 1.0);
    }

    public void setShootPercent(double percent, double spinRatio) {
        shooter1PidController.setReference(percent * spinRatio, ControlType.kDutyCycle);
        shooter2PidController.setReference(percent, ControlType.kDutyCycle);
    } 

    public void setIndexerPercent(double percent){
        indexerPidController.setReference(percent, ControlType.kDutyCycle);
    }

    @Override
    public void periodic(){
        updateShooterTelemetry();
        if(true)
            return;
        switch (currentState){
            case IDLE:
                if(stateSwitched){
                    stateSwitched = false;

                    if(pieceDetected()){
                        hasPiece = true;
                    }

                    //executed when enter state
                     if(this.getCurrentCommand() != this.getDefaultCommand()){
                        setShooterVelocity(0);
                        setIndexerVelocity(0);
                     }
                }
                
                    currentState = TurbotakeState.INTAKE;
                    stateSwitched = true;
                break;

            case INTAKE:
                if(stateSwitched){
                    stateSwitched = false;

                    if(this.getCurrentCommand() != this.getDefaultCommand()){
                        setIndexerVelocity(INDEXER_SPEED);
                    }    
                }
                if(hasPiece == true){
                    currentState = TurbotakeState.IDLE;
                    setIndexerVelocity(0);
                    stateSwitched = true;
                }
                break;

            case AMP_SHOOT:
                if(stateSwitched){
                    stateSwitched = false;

                    if(this.getCurrentCommand() != this.getDefaultCommand()){
                        setIndexerVelocity(-AMP_SPEED);
                        
                    }
                }

                if(hasPiece == false){
                    setIndexerVelocity(0);
                    currentState = TurbotakeState.SPEAKER_SHOOT;
                    stateSwitched = true;
                }
            break;

            case SPEAKER_SHOOT:
                if(stateSwitched){
                    stateSwitched = false;

                    if(this.getCurrentCommand() != this.getDefaultCommand()){
                       setShooterVelocity(SPEAKER_SPEED);
                    }
                }

                if(shooter1Encoder.getVelocity() == 1200){
                    setIndexerVelocity(INDEXER_SPEED);
                }

                if(hasPiece == false){
                    currentState = TurbotakeState.TRAP_SHOOT;
                    stateSwitched = true;
                }

                if(stateSwitched){
                    setIndexerVelocity(0);
                    setShooterVelocity(0);
                }
                break;
            case TRAP_SHOOT:
                if(stateSwitched){
                    stateSwitched = false;

                    if(this.getCurrentCommand() != this.getDefaultCommand()){
                        setIndexerVelocity(-INDEXER_SPEED);
                    }
                }

                if(hasPiece == false){
                    currentState = TurbotakeState.IDLE;
                    stateSwitched = true;
                }

                if(stateSwitched){
                    setIndexerVelocity(0);
                }
            break;
        }
    }

    public void updateShooterTelemetry(){
        beamBreakBooleanPublisher.set(pieceDetected());
        shooter1RPMPublisher.set(shooter1Encoder.getVelocity());
        shooter2RPMPublisher.set(shooter2Encoder.getVelocity());
        indexerRPMPublisher.set(indexerEncoder.getVelocity());
    }


    public Command getIndexerSysID(){
        return Commands.sequence(
                indexerRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5), 
                indexerRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                new WaitCommand(5),
                indexerRoutine.dynamic(SysIdRoutine.Direction.kForward),
                new WaitCommand(5),
                indexerRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
        
    }

    public Command getShooterSysID(){
        return Commands.sequence(
            shooterRoutine.quasistatic(SysIdRoutine.Direction.kForward),
            new WaitCommand(5), 
            shooterRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
            new WaitCommand(5),
            shooterRoutine.dynamic(SysIdRoutine.Direction.kForward),
            new WaitCommand(5),
            shooterRoutine.dynamic(SysIdRoutine.Direction.kReverse)
            );
    }

}