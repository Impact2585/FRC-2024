package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Shooter extends SubsystemBase {

    private Spark shooterTopMotor = new Spark(ShooterConstants.shooterTopPort);
    private Spark shooterBottomMotor = new Spark(ShooterConstants.shooterBottomPort);

    private double shooterStatus;

    public Shooter() {
        shooterStatus = 0.0;
    }

    @Override
    public void periodic() {
    }

    public void goBackwards() {
        //shooterStatus = 1.0;
        shooterTopMotor.set(ShooterConstants.ampSpeed);
        shooterBottomMotor.set(-ShooterConstants.ampSpeed);
        System.out.println("Amp speed");
    }

    public void speakerScoring(){
        //shooterStatus = 2.0;
        shooterTopMotor.set(-ShooterConstants.speakerSpeed);
        shooterBottomMotor.set(ShooterConstants.speakerSpeed);
        System.out.println("Speaker speed");
    }

    public void stopShooter(){
        shooterTopMotor.set(0);
        shooterBottomMotor.set(0);
    }
}