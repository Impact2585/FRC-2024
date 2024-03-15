package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends SubsystemBase {

    private CANSparkMax intakeLowerMotor = new CANSparkMax(IntakeConstants.lowerIntakeMotorCanId, MotorType.kBrushless);
    private CANSparkMax intakeUpperMotor = new CANSparkMax(IntakeConstants.upperIntakeMotorCanId, MotorType.kBrushless);

    private double intakeStatus;

    public Intake() {
        intakeLowerMotor.setSmartCurrentLimit(20);
        intakeUpperMotor.setSmartCurrentLimit(20);
        stopIntake();
    }

    @Override
    public void periodic() {

    }

    public void spinIn() {
        System.out.println("spinning");
        intakeLowerMotor.set(IntakeConstants.maxSpeed);
        intakeUpperMotor.set(IntakeConstants.maxSpeed);
    }

    public void spinOut(){
        System.out.println("spinning out");
        intakeLowerMotor.set(-IntakeConstants.maxSpeed);
        intakeUpperMotor.set(-IntakeConstants.maxSpeed);
    }

    public void stopIntake(){
        intakeLowerMotor.set(0);
        intakeUpperMotor.set(0);
    }
}