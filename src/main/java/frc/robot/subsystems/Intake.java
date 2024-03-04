package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeLeftMotor = new CANSparkMax(IntakeConstants.leftIntakeMotorCanId, MotorType.kBrushless);
    private CANSparkMax intakeRightMotor = new CANSparkMax(IntakeConstants.rightIntakeMotorCanId, MotorType.kBrushless);

    private double intakeStatus;

    public Intake() {
        intakeLeftMotor.setSmartCurrentLimit(20);
        intakeRightMotor.setSmartCurrentLimit(20);
        stopIntake();
    }

    @Override
    public void periodic() {

    }

    public void spinIn() {
        intakeLeftMotor.set(IntakeConstants.maxSpeed);
        intakeRightMotor.set(-IntakeConstants.maxSpeed);
    }

    public void spinOut(){
        intakeLeftMotor.set(-IntakeConstants.maxSpeed);
        intakeRightMotor.set(IntakeConstants.maxSpeed);
    }

    public void stopIntake(){
        intakeLeftMotor.set(0);
        intakeRightMotor.set(0);
    }
}