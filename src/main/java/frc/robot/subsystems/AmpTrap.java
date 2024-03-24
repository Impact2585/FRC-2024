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
    double status = 1.0;
    boolean locked = true;

    public AmpTrap() {
        elevatorEncoder.setPosition(1);
        elevatorMotor.setInverted(true);
        //elevatorEncoder.setInverted(true);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorMotor.setSmartCurrentLimit(20);
        rollerMotor.setSmartCurrentLimit(20);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Elevator position", elevatorEncoder.getPosition());

        SmartDashboard.putBoolean("Elevator locked?", locked);
        
        if(elevatorEncoder.getPosition() > AmptrapConstants.elevatorHighStop) {
            canUp = false;
            if(status != 2 && locked) elevatorMotor.set(0);
            if(status != 2 && locked) System.out.println("Elevator at high point");
            status = 2.0;
        }
        else {
            canUp = true;
            status = 1.0;
        }

        SmartDashboard.putBoolean("Elevator can go up?", canUp);
        
        if(elevatorEncoder.getPosition() < AmptrapConstants.elevatorLowStop) {
            canDown = false;
            if(status != 0 && locked) elevatorMotor.set(0);
            if(status != 0 && locked) System.out.println("Elevator at low point");
            status = 0.0;
        }
        else {
            canDown = true;
            status = 1.0;
        }

        SmartDashboard.putBoolean("Elevator can go down?", canDown);

        SmartDashboard.putNumber("Elevator status", status);
    }

    public void setMotor(double speed){
        elevatorMotor.set(speed);
    }

    public void raiseElevator() {
        if(canUp || !locked) elevatorMotor.set(AmptrapConstants.elevatorSpeed);
        System.out.println("Elevator raising");
    }

    public void lowerElevator(){
        if(canDown || !locked) elevatorMotor.set(-AmptrapConstants.elevatorSpeed);
        System.out.println("Elevator lowering");
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

    public double getEncoderPos() {
        return elevatorEncoder.getPosition();
    }

    public void unlock(){
        locked = false;
        //elevatorEncoder.setPosition(1);
    }

    public void lock(){
        locked = true;
        elevatorEncoder.setPosition(1);
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
