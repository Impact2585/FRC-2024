package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.AmptrapConstants;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Pivot extends SubsystemBase {

    CANSparkMax pivotMotor = new CANSparkMax(PivotConstants.PivotMotorCanId, MotorType.kBrushless);
    RelativeEncoder pivotEncoder;
    boolean locked = false;
    boolean canUp = true;
    boolean canDown = true;
    double status = 1.0;

    public Pivot() {
        pivotEncoder = pivotMotor.getEncoder();
        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(40);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot position", pivotEncoder.getPosition());
        
        SmartDashboard.putBoolean("Pivot locked?", locked);
        
        if(pivotEncoder.getPosition() > PivotConstants.highLimit) {
            canUp = false;
            if(status != 2.0 && locked) pivotMotor.set(0);
            if(status != 2.0 && locked) System.out.println("Pivot at high point");
            status = 2.0;
        }
        else {
            canUp = true;
            status = 1.0;
        }

        SmartDashboard.putBoolean("Pivot can go up?", canUp);
        
        if(pivotEncoder.getPosition() < PivotConstants.lowLimit) {
            canDown = false;
            if(status != 0.0 && locked) pivotMotor.set(0);
            if(status != 0.0 && locked) System.out.println("Elevator at low point");
            status = 0.0;
        }
        else {
            canDown = true;
            status = 1.0;
        }

        SmartDashboard.putBoolean("Pivot can go down?", canDown);

        SmartDashboard.putNumber("Pivot status", status);
        
    }

    public void setMotor(double speed){
        if(Math.abs(speed) > PivotConstants.pivotSpeed) speed = sign(speed) * PivotConstants.pivotSpeed;
        pivotMotor.set(speed);
    }

    public double sign(double x){
        if(x < 0) return -1;
        else return 1;
    }

    public void raisePivot() {
        pivotMotor.set(PivotConstants.pivotSpeed);
        System.out.println("Pivot raising");
    }

    public void lowerPivot(){
        pivotMotor.set(-PivotConstants.pivotSpeed);
        System.out.println("Pivot lowering");
    }

    public void stopPivot(){
        pivotMotor.set(0);
    }

    public double getEncoderMeters() {
        return pivotEncoder.getPosition();
    }
}
