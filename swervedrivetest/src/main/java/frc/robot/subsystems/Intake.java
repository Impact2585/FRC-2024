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
        intakeStatus = 0.0;
        intakeLeftMotor.setSmartCurrentLimit(20);
        intakeRightMotor.setSmartCurrentLimit(20);
    }

    @Override
    public void periodic() {
        intakeLeftMotor.set(-IntakeConstants.maxSpeed * intakeStatus);
        intakeRightMotor.set(IntakeConstants.maxSpeed * intakeStatus);
    }

    public void spinIn() {
        if(intakeStatus != 1.0){
            intakeStatus = 1.0;
            System.out.println("Intake spinning inward");
        }
        else{
            intakeStatus = 0.0;
            System.out.println("Intake stopped");
        }
    }

    public void spinOut(){
        if(intakeStatus != -1.0){
            intakeStatus = -1.0;
            System.out.println("Intake outtaking!");
        }
        else{
            intakeStatus = 0.0;
            System.out.println("Intake stopped");
        }
    }
}