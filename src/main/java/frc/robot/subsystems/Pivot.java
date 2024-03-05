package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
    AbsoluteEncoder pivotEncoder;

    public Pivot() {
        pivotEncoder = pivotMotor.getAbsoluteEncoder(Type.kDutyCycle);
        //pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);
        pivotMotor.setSmartCurrentLimit(20);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot position", pivotEncoder.getPosition());
    }

    public void setMotor(double speed){
        pivotMotor.set(speed);
    }

    public void raisePivot() {
        pivotMotor.set(Math.min(
            Math.abs(PivotConstants.highLimit - pivotEncoder.getPosition()) * PivotConstants.pivotLimitSlowdownFactor,
            PivotConstants.pivotSpeed));
    }

    public void lowerPivot(){
        pivotMotor.set(-Math.min(
            Math.abs(PivotConstants.lowLimit - pivotEncoder.getPosition()) * PivotConstants.pivotLimitSlowdownFactor,
            PivotConstants.pivotSpeed));
    }

    public void stopPivot(){
        pivotMotor.set(0);
    }

    public double getEncoderMeters() {
        return pivotEncoder.getPosition();
    }
}
