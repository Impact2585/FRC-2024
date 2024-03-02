package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Shooter extends SubsystemBase {

    private Talon shooterLeftMotor = new Talon(ShooterConstants.shooterLeftPort);
    private Talon shooterRightMotor = new Talon(ShooterConstants.shooterRightPort);

    private double shooterStatus;

    public Shooter() {
        shooterStatus = 0.0;
    }

    @Override
    public void periodic() {
    }

    public void ampScoring() {
        if(shooterStatus != 1.0){
            shooterStatus = 1.0;
            shooterLeftMotor.set(ShooterConstants.ampSpeed);
            shooterRightMotor.set(-ShooterConstants.ampSpeed);
            System.out.println("Amp speed");
        }
        else{
            shooterStatus = 0.0;
            this.stop();
            System.out.println("Shooter stopped");
        }
    }

    public void speakerScoring(){
        if(shooterStatus != 2.0){
            shooterStatus = 2.0;
            shooterLeftMotor.set(ShooterConstants.speakerSpeed);
            shooterRightMotor.set(-ShooterConstants.speakerSpeed);
            System.out.println("Speaker speed");
        }
        else{
            shooterStatus = 0.0;
            this.stop();
            System.out.println("Shooter stopped");
        }
    }

    public void stop(){
        shooterLeftMotor.set(0);
        shooterRightMotor.set(0);
    }
}