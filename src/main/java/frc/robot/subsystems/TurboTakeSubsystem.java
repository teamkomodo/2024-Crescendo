/*Turbotake subsystem code
*Intake/Shooter
* 2 Shooter Motors
* 1 indexer motor
*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.INDEXER_MOTOR_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_ID1;
import static frc.robot.Constants.SHOOTER_MOTOR_ID2;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurboTakeSubsystem extends SubsystemBase{
     //defines motors
     private CANSparkMax shooterMotor1;
     private CANSparkMax indexerMotor;
     private CANSparkMax shooterMotor2;

     //init outtake motors and restores factory defaults
     public void ShootingSubsystem(){
        shooterMotor1 = new CANSparkMax(SHOOTER_MOTOR_ID1, MotorType.kBrushless);
        shooterMotor2 = new CANSparkMax(SHOOTER_MOTOR_ID2, MotorType.kBrushless);
       
        shooterMotor2.restoreFactoryDefaults();
        shooterMotor1.restoreFactoryDefaults();
     }

     //init intake motor and restores factory defaults
     public void IndexerSubsystem(){
        indexerMotor = new CANSparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);

        indexerMotor.restoreFactoryDefaults();
     }

     //Turns on shooter motor at 12 volts * shooterPercent
     public  void SetShooterSpeed(double shooterPercent){
        shooterMotor1.set(shooterPercent);
        shooterMotor2.set(shooterPercent);
     }

     //Turns on indexer motor at 12 volts * indexerPercent
     public  void SetIndexerSpeed(double indexerPercent){
        indexerMotor.set(indexerPercent);
     }


}