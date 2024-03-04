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
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class PivotPID extends CommandBase {
    private final Pivot pivotSubsystem;
    private final PIDController pivotPid;

    public PivotPID(Pivot pivotSubsystem, double setpoint) {
        this.pivotSubsystem = pivotSubsystem;
        this.pivotPid = new PIDController(//
                PivotConstants.pivotP, PivotConstants.pivotI, PivotConstants.pivotD);
        pivotPid.setSetpoint(setpoint);

        addRequirements(pivotSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("PivotPID started!");
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
        return false;
    }
}
