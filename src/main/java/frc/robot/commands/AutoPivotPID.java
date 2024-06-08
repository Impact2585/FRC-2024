package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.PivotConstants;

import frc.robot.subsystems.Pivot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class AutoPivotPID extends Command {
    private final Pivot pivotSubsystem;
    private final PIDController pivotPid;
    private final Timer t;

    public AutoPivotPID(Pivot pivotSubsystem, double setpoint) {
        this.pivotSubsystem = pivotSubsystem;
        this.pivotPid = new PIDController(//
                PivotConstants.pivotP, PivotConstants.pivotI, PivotConstants.pivotD);
        pivotPid.setSetpoint(setpoint);
        this.pivotPid.setTolerance(0.1);
        addRequirements(pivotSubsystem);
        t = new Timer();
        
    }

    @Override
    public void initialize() {
        System.out.println("PivotPID started!");
        t.start();
        pivotPid.reset();
    }

    @Override
    public void execute() {
        double speed = pivotPid.calculate(pivotSubsystem.getEncoderMeters());
        pivotSubsystem.setMotor(speed);
    }

    @Override
    public void end(boolean interrupted) {
        pivotSubsystem.setMotor(0);
        System.out.println("PivotPID ended!");
    }

    @Override
    public boolean isFinished() {
        if(t.get() > 2) return true;
        return false;
    }
}
