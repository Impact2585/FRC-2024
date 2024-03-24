package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Constants.AmptrapConstants;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SubsystemBase {
    CANSparkMax leftClimber = new CANSparkMax(ClimbConstants.leftClimberCanID, MotorType.kBrushless);
    RelativeEncoder leftEncoder = leftClimber.getEncoder();
    CANSparkMax rightClimber = new CANSparkMax(ClimbConstants.rightClimberCanID, MotorType.kBrushless);
    RelativeEncoder rightEncoder = rightClimber.getEncoder();
    boolean canUp = true;
    boolean canDown = true;
    double status = 1;
    boolean locked = false;

    public Climb() {
        leftEncoder.setPosition(1);
        rightEncoder.setPosition(1);
        leftClimber.setSmartCurrentLimit(20);
        rightClimber.setSmartCurrentLimit(20);
        rightClimber.setIdleMode(IdleMode.kBrake);
        leftClimber.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climb average position", climbPos());
        SmartDashboard.putNumber("Left arm position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right arm position", -rightEncoder.getPosition());
        
        if(climbPos() > AmptrapConstants.elevatorHighStop) {
            canUp = false;
            if(status != 2 && locked) stopClimb();
            if(status != 2 && locked) System.out.println("Climb at high point");
            status = 2.0;
        }
        else {
            canUp = true;
            status = 1.0;
        }

        SmartDashboard.putBoolean("Climb can go up?", canUp);
        
        if(climbPos() < AmptrapConstants.elevatorLowStop) {
            canDown = false;
            if(status != 0 && locked) stopClimb();
            if(status != 0 && locked) System.out.println("Climb at low point");
            status = 0.0;
        }
        else {
            canDown = true;
            status = 1.0;
        }

        SmartDashboard.putBoolean("Climb can go down?", canDown);

        SmartDashboard.putNumber("Climb status", status);
    }

    public void raiseClimb() {
        if(canUp || !locked){
            System.out.println("Lifting arms...");
            leftClimber.set(ClimbConstants.climbSpeed);
            rightClimber.set(-ClimbConstants.climbSpeed);
        }
        
    }

    public void lowerClimb(){
        if(canDown || !locked){
            System.out.println("Pulling up...");
            leftClimber.set(-ClimbConstants.climbSpeed);
            rightClimber.set(ClimbConstants.climbSpeed);
        }   
    }

    public void stopClimb(){
        leftClimber.set(0);
        rightClimber.set(0);
    }

    public double climbPos(){
        return (leftEncoder.getPosition() - rightEncoder.getPosition()) / 2;
    }

    public void unlockClimb(){
        locked = false;
    }

    public void lockClimb(){
        locked = true;
        leftEncoder.setPosition(1);
        rightEncoder.setPosition(-1);
    }
}
