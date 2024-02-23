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
    private final CANSparkMax leftShooterMotor;
    private final CANSparkMax rightShooterMotor;
    private final CANSparkMax indexerMotor;
    
    //Encoders
    private final RelativeEncoder leftShooterEncoder;
    private final RelativeEncoder rightShooterEncoder;
    private final RelativeEncoder indexerEncoder;
    
    //PID Controllers
    private final SparkPIDController indexerPidController;
    private final SparkPIDController leftShooterPidController;
    private final SparkPIDController rightShooterPidController;
    
    //defines beam break sensor
    private final DigitalInput turbotakeNoteSensor;
    
    //Telemetry
    
    //motor telemetry
    private final DoublePublisher leftShooterVelocityPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("leftshootervelocity").publish();
    private final DoublePublisher rightShooterVelocityPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("rightshootervelocity").publish();
    private final DoublePublisher indexerVelocityPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("indexervelocity").publish();
    
    //beam break sensor telemetry
    private final BooleanPublisher pieceDetectedPublisher = NetworkTableInstance.getDefault().getBooleanTopic("piecedetected").publish();
    
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
        shooterFF = 0.0023097;
        shooterMinOutput = -1;
        shooterMaxOutput = 1;
        
        //Initialize the motors
        leftShooterMotor = new CANSparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
        
        //inverts motors to correct orientation
        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true);
        indexerMotor.setInverted(true);

        //Initialize the beam break sensor
        turbotakeNoteSensor = new DigitalInput(TURBOTAKE_NOTE_SENSOR_PORT);
        
        //Initializes encoders
        leftShooterEncoder = leftShooterMotor.getEncoder();
        rightShooterEncoder = rightShooterMotor.getEncoder();
        indexerEncoder = indexerMotor.getEncoder();
        
        //sets encoder positions to 0
        leftShooterEncoder.setPosition(0);
        rightShooterEncoder.setPosition(0);
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
        leftShooterPidController = leftShooterMotor.getPIDController();
        rightShooterPidController = rightShooterMotor.getPIDController();
        
        //sets PID values for left shooter
        leftShooterPidController.setP(shooterP);
        leftShooterPidController.setI(shooterI);
        leftShooterPidController.setD(shooterD);
        leftShooterPidController.setIAccum(shooterIAccumulator);
        leftShooterPidController.setFF(shooterFF);
        leftShooterPidController.setOutputRange(shooterMinOutput, shooterMaxOutput);
        
        //sets PID values for right shooter
        rightShooterPidController.setP(shooterP);
        rightShooterPidController.setI(shooterI);
        rightShooterPidController.setD(shooterD);
        rightShooterPidController.setIAccum(shooterIAccumulator);
        rightShooterPidController.setFF(shooterFF);
        rightShooterPidController.setOutputRange(shooterMinOutput, shooterMaxOutput);
        
    }

    @Override
    public void periodic(){
        updateShooterTelemetry();
    }
    
    public void updateShooterTelemetry(){
        pieceDetectedPublisher.set(isPieceDetected());
        leftShooterVelocityPublisher.set(leftShooterEncoder.getVelocity());
        rightShooterVelocityPublisher.set(rightShooterEncoder.getVelocity());
        indexerVelocityPublisher.set(indexerEncoder.getVelocity());
    }
    
    // Returns true if a piece has triggered the beambreak
    public boolean isPieceDetected(){
        // The sensor returns false when the beam is broken
        return !turbotakeNoteSensor.get();
    }

    public double getShooterVelocity() {
        return rightShooterEncoder.getVelocity();
    }

    public double getIndexerVelocity() {
        return indexerEncoder.getVelocity();
    }
    
    // commands the shooter to a target velocity
    public void setShooterVelocity(double velocity){
        leftShooterPidController.setReference(velocity * SPIN_RATIO, CANSparkMax.ControlType.kVelocity);
        rightShooterPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
    
    // commands the indexer to a target velocity
    public void setIndexerVelocity(double velocity){
        indexerPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    }
    
    public void setShooterPercent(double percent){
        setShootPercent(percent, 1.0);
    }
    
    public void setShootPercent(double percent, double spinRatio) {
        leftShooterPidController.setReference(percent * spinRatio, ControlType.kDutyCycle);
        rightShooterPidController.setReference(percent, ControlType.kDutyCycle);
    }
    
    public void setIndexerPercent(double percent){
        indexerPidController.setReference(percent, ControlType.kDutyCycle);
    }
    
    // Commands

    public Command indexerSysIdCommand(){
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
    
    public Command shooterSysIdCommand(){
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

    public Command spinUpFlywheelCommand(double speed, double threshold) {
        // Command will run until the flywheel has spun up
        return Commands.run(() -> setShooterVelocity(speed), this).until(() -> (rightShooterEncoder.getVelocity() - speed < threshold));
    }
    
}