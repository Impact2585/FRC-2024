package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.AmptrapConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AmpTrap extends SubsystemBase {
    CANSparkMax elevatorMotor = new CANSparkMax(AmptrapConstants.elevatorCanID, MotorType.kBrushless);
    RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();
    CANSparkMax rollerMotor = new CANSparkMax(AmptrapConstants.rollerCanID, MotorType.kBrushless);
    boolean canUp = true;
    boolean canDown = true;

    public AmpTrap() {
        elevatorEncoder.setPosition(-5);
        elevatorMotor.setInverted(true);
        //elevatorEncoder.setInverted(true);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(20);
        rollerMotor.setSmartCurrentLimit(20);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator position", elevatorEncoder.getPosition());
        /* 
        if(elevatorEncoder.getPosition() < AmptrapConstants.elevatorHighStop) canUp = false;
        else canUp = true;
        SmartDashboard.putBoolean("Elevator can go up?", canUp);

        if(elevatorEncoder.getPosition() > AmptrapConstants.elevatorLowStop) canDown = false;
        else canDown = true;
        SmartDashboard.putBoolean("Elevator can go down?", canDown);
        */
    }

    public void setMotor(double speed){
        elevatorMotor.set(speed);
    }

    public void raiseElevator() {
        //if(canUp) elevatorMotor.set(AmptrapConstants.elevatorSpeed);
        elevatorMotor.set(AmptrapConstants.elevatorSpeed);
    }

    public void lowerElevator(){
        //if(canDown) elevatorMotor.set(-AmptrapConstants.elevatorSpeed);
        elevatorMotor.set(-AmptrapConstants.elevatorSpeed);
    }

    public void intakeRoller(){
        rollerMotor.set(-AmptrapConstants.rollerSpeed);
    }

    public void outtakeRoller(){
        rollerMotor.set(AmptrapConstants.rollerSpeed);
    }

    public void stopRoller(){
        rollerMotor.set(0);
    }
    
    public void stopElevator(){
        elevatorMotor.set(0);
    }

    public double getEncoderMeters() {
        return 0.0;
        //return elevatorEncoder.getPosition();
    }

    public void resetAmptrapEncoder(){
        elevatorEncoder.setPosition(-5);
    }

    public void score(){
        elevatorMotor.set(AmptrapConstants.elevatorSpeed);
        new WaitCommand(0.5);
        rollerMotor.set(AmptrapConstants.rollerSpeed);
    }

    public void stopAll(){
        elevatorMotor.set(0);
        rollerMotor.set(0);
    }
}
