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
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
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
    private final DigitalInput beamBreakSensor;
    
    //Telemetry
    
    //motor telemetry
    private final DoublePublisher leftShooterVelocityPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("leftshootervelocity").publish();
    private final DoublePublisher rightShooterVelocityPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("rightshootervelocity").publish();
    private final DoublePublisher indexerVelocityPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("indexervelocity").publish();
    private final DoublePublisher filteredCurrentPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("filteredcurrent").publish();

    //beam break sensor telemetry
    private final BooleanPublisher pieceDetectedPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getBooleanTopic("piecedetected").publish();
    private final BooleanPublisher hasPiecePublisher = NetworkTableInstance.getDefault().getTable("turbotake").getBooleanTopic("haspiece").publish();

    private final StringPublisher currentCommandPublisher = NetworkTableInstance.getDefault().getTable("turbotake").getStringTopic("currentcommand").publish();

    private final DoubleEntry shooterSpeedEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("shooterspeed").getEntry(CLOSE_SHOOTER_SPEED);

    private final DoubleEntry indexerPEntry, indexerIEntry, indexerDEntry, shooterPEntry, shooterIEntry, shooterDEntry, shooterFFEntry;

    //PID values for indexer
    private double indexerP, indexerI, indexerD, indexerIZone, indexerFF, indexerMinOutput, indexerMaxOutput;
    //PID values for shooter motors
    private double shooterP, shooterI, shooterD, shooterIZone, shooterFF, shooterMinOutput, shooterMaxOutput;

    private boolean pieceLoaded = false;
    private boolean pieceLoadedAtLastCheck = false;
    
    
    public final SysIdRoutine shooterRoutine = new SysIdRoutine(
            new SysIdRoutine.Config(), 
            new SysIdRoutine.Mechanism(
            (voltage) -> setShooterVelocity(voltage.in(Units.Volts)),
            null,
            this
    ));

    private double filteredCurrent = 0;
    private double currentFilterConstant = 0.1;

    private boolean pieceDetected = false;
    private double beambreakPasses = 0;
    
    public TurbotakeSubsystem(){

        // PID coefficients for indexer
        indexerP = 1;
        indexerI = 0;
        indexerD = 0;
        indexerIZone = 0;
        indexerFF = 0;
        indexerMinOutput = -1;
        indexerMaxOutput = 1;

        indexerPEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/indexerkP").getEntry(indexerP);
        indexerIEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/indexerkI").getEntry(indexerI);
        indexerDEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/indexerkD").getEntry(indexerD);

        indexerPEntry.set(indexerP);
        indexerIEntry.set(indexerI);
        indexerDEntry.set(indexerD);
        
        // PID coefficients for shooter motors
        shooterP = 0.001;
        shooterI = 0;
        shooterD = 0;
        shooterIZone = 1;
        shooterFF = 0.00022;
        shooterMinOutput = -1;
        shooterMaxOutput = 1;

        shooterPEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/shooterkP").getEntry(shooterP);
        shooterIEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/shooterkI").getEntry(shooterI);
        shooterDEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/shooterkD").getEntry(shooterD);
        shooterFFEntry = NetworkTableInstance.getDefault().getTable("turbotake").getDoubleTopic("tuning/shooterkV").getEntry(shooterFF);

        shooterPEntry.set(shooterP);
        shooterIEntry.set(shooterI);
        shooterDEntry.set(shooterD);
        shooterFFEntry.set(shooterFF);

        //Initialize the motors
        leftShooterMotor = new CANSparkMax(LEFT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        rightShooterMotor = new CANSparkMax(RIGHT_SHOOTER_MOTOR_ID, MotorType.kBrushless);
        indexerMotor = new CANSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
        
        //inverts motors to correct orientation
        leftShooterMotor.setInverted(false);
        rightShooterMotor.setInverted(true);
        indexerMotor.setInverted(true);

        //Initialize the beam break sensor
        beamBreakSensor = new DigitalInput(INTAKE_BEAM_BREAK_PORT);
        
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
        indexerPidController.setIZone(indexerIZone);
        indexerPidController.setFF(indexerFF);
        indexerPidController.setOutputRange(indexerMinOutput, indexerMaxOutput);
        
        //initializes shooter1 and shooter2 PID controllers to the PID controllers in each motor
        leftShooterPidController = leftShooterMotor.getPIDController();
        rightShooterPidController = rightShooterMotor.getPIDController();
        
        //sets PID values for left shooter
        leftShooterPidController.setP(shooterP);
        leftShooterPidController.setI(shooterI);
        leftShooterPidController.setD(shooterD);
        leftShooterPidController.setIZone(shooterIZone);
        leftShooterPidController.setFF(shooterFF);
        leftShooterPidController.setOutputRange(shooterMinOutput, shooterMaxOutput);

        leftShooterMotor.setSmartCurrentLimit(50);
        rightShooterMotor.setSmartCurrentLimit(50);
        
        //sets PID values for right shooter
        rightShooterPidController.setP(shooterP);
        rightShooterPidController.setI(shooterI);
        rightShooterPidController.setD(shooterD);
        rightShooterPidController.setIZone(shooterIZone);
        rightShooterPidController.setFF(shooterFF);
        rightShooterPidController.setOutputRange(shooterMinOutput, shooterMaxOutput);
        
        shooterSpeedEntry.set(CLOSE_SHOOTER_SPEED);
    }

    public void teleopInit() {
        leftShooterMotor.set(0);
        rightShooterMotor.set(0);
        indexerMotor.set(0);
    }

    @Override
    public void periodic() {
        updateShooterTelemetry();
        filterCurrent();
        countBeambreakPasses();
        updateControlConstants();
    }

    private void filterCurrent() {
        filteredCurrent = filteredCurrent * (1 - currentFilterConstant) + indexerMotor.getOutputCurrent() * currentFilterConstant;
    }

    private void countBeambreakPasses() {        
        if (pieceDetected != isPieceDetected()) {
            pieceDetected = isPieceDetected();
            beambreakPasses += 1 * Math.signum(indexerEncoder.getVelocity());
        }
    }
    
    public void updateShooterTelemetry(){
        pieceDetectedPublisher.set(isPieceDetected());
        leftShooterVelocityPublisher.set(leftShooterEncoder.getVelocity());
        rightShooterVelocityPublisher.set(rightShooterEncoder.getVelocity());
        indexerVelocityPublisher.set(indexerEncoder.getVelocity());
        filteredCurrentPublisher.set(filteredCurrent);
        hasPiecePublisher.set(hasPiece());
        currentCommandPublisher.set(getCurrentCommand() != null? getCurrentCommand().getName() : "null");
    }

    public void updateControlConstants() {

        if(!TUNING_MODE)
            return;

        double newIndexerP = indexerPEntry.get(indexerP);
        if(newIndexerP != indexerP) {
            indexerP = newIndexerP;
            indexerPidController.setP(indexerP);
        }

        double newIndexerI = indexerIEntry.get(indexerI);
        if(newIndexerI != indexerI) {
            indexerI = newIndexerI;
            indexerPidController.setI(indexerI);
        }

        double newIndexerD = indexerDEntry.get(indexerD);
        if(newIndexerD != indexerD) {
            indexerD = newIndexerD;
            indexerPidController.setD(indexerD);
        }

        double newShooterP = shooterPEntry.get(shooterP);
        if(newShooterP != shooterP) {
            shooterP = newShooterP;
            leftShooterPidController.setP(shooterP);
            rightShooterPidController.setP(shooterP);
        }

        double newShooterI = shooterIEntry.get(shooterI);
        if(newShooterI != shooterI) {
            shooterI = newShooterI;
            leftShooterPidController.setI(shooterI);
            rightShooterPidController.setI(shooterI);
        }

        double newShooterD = shooterDEntry.get(shooterD);
        if(newShooterD != shooterD) {
            shooterD = newShooterD;
            leftShooterPidController.setD(shooterD);
            rightShooterPidController.setD(shooterD);
        }

        double newShooterFF = shooterFFEntry.get(shooterFF);
        if(newShooterFF != shooterFF) {
            shooterFF = newShooterFF;
            leftShooterPidController.setFF(shooterFF);
            rightShooterPidController.setFF(shooterFF);
        }

    }
    
    /**
     * @return If the beambreak sensor is triggered
     */
    public boolean isPieceDetected(){
        // The sensor returns false when the beam is broken
        return !beamBreakSensor.get();
    }

    /**
     * By counting the number of times the beam break sensor is triggered, we can determine if the turbotake has a piece even if the beam break is not currently triggered.
     * 
     * @return If the turbotake thinks it has a piece
     */
    public boolean hasPiece() {
        return beambreakPasses % 2 == 1 || isPieceDetected();
    }

    public double getShooterVelocity() {
        return rightShooterEncoder.getVelocity();
    }

    public double getIndexerVelocity() {
        return indexerEncoder.getVelocity();
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
    }
    
    // commands the shooter to a target velocity
    public void setShooterVelocity(double velocity){
        System.out.println("RUNNING SHOOTERS: " + velocity);
        leftShooterPidController.setReference(velocity * SPIN_RATIO, CANSparkMax.ControlType.kVelocity);
        rightShooterPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);

    }
    
    // commands the indexer to a target velocity
    // public void setIndexerVelocity(double velocity){
    //     System.out.println("RUNNING INDEXER");
    //     indexerPidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    // }
    
    public void setShooterPercent(double percent){
        setShooterPercent(percent, 1.0);
    }
    
    public void setShooterPercent(double percent, double spinRatio) {
        System.out.println("RUNNING SHOOTERS, SPIN RATIO " + " " + spinRatio);
        leftShooterPidController.setReference(percent * spinRatio, ControlType.kDutyCycle);
        rightShooterPidController.setReference(percent, ControlType.kDutyCycle);
        
    }
    
    public void setIndexerPercent(double percent){
        System.out.println("RUNNING INDEXER, SPEED " + " " + percent);
        indexerPidController.setReference(percent, ControlType.kDutyCycle);
    }

    //turns off shooter and sets Iaccum to 0 to reset I term
    public void turnoffShooter(){
        setShooterPercent(0);
        leftShooterPidController.setIAccum(0);
        rightShooterPidController.setIAccum(0);
    }

    //turn off indexer and sets Iaccum to 0 to reset I term
    public void turnOffIndexer(){
        setIndexerPercent(0);
        indexerPidController.setIAccum(0);

    }

    public void checkNoteIndexer(){
        pieceLoadedAtLastCheck = pieceLoaded;
        pieceLoaded = isPieceDetected();
        if(!pieceLoadedAtLastCheck && pieceLoaded){
            turnOffIndexer();
        }
    }

    public boolean checkShooterSpeed(double targetSpeed, double tolerance){
        return Math.abs(rightShooterEncoder.getVelocity() - targetSpeed) < tolerance;
    }

    public double getShooterSpeed(){
        return !TUNING_MODE? CLOSE_SHOOTER_SPEED : shooterSpeedEntry.get(CLOSE_SHOOTER_SPEED);
    }
    
    // Command 
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

    public Command shootForSpeaker(){
        return Commands.sequence(
            Commands.runOnce(() -> setShooterVelocity(SPEAKER_SPEED)),
            Commands.waitSeconds(2.0),
            Commands.runOnce(() -> setIndexerPercent(1)),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> turnOffIndexer()),
            Commands.runOnce(() -> turnoffShooter())
        ).finallyDo(() -> {
            turnOffIndexer();
            turnoffShooter();
        });
    }

    public Command shootForAmp(){
        return Commands.sequence(
            Commands.waitUntil(() -> !isPieceDetected()),
            Commands.runOnce(() -> setIndexerPercent(-1)),
            Commands.waitSeconds(1),
            Commands.runOnce(() -> turnOffIndexer())
        ).finallyDo(() -> {
            turnOffIndexer();
        });
    }
}