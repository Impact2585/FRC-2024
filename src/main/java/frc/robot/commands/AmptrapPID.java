package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.PivotConstants;

import frc.robot.subsystems.AmpTrap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class AmptrapPID extends Command {
    private AmpTrap amptrapSubsystem;
    private PIDController elevatorPID;

    public AmptrapPID(AmpTrap amptrapSubsystem, double setpoint) {
        this.amptrapSubsystem = amptrapSubsystem;
        this.elevatorPID = new PIDController(//
                PivotConstants.pivotP, PivotConstants.pivotI, PivotConstants.pivotD);
        this.elevatorPID.setTolerance(1);
        elevatorPID.setSetpoint(setpoint);

        addRequirements(amptrapSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Amptrap PID started!");
        elevatorPID.reset();
    }

    @Override
    public void execute() {
        double speed = elevatorPID.calculate(amptrapSubsystem.getEncoderPos(), elevatorPID.getSetpoint());
        amptrapSubsystem.setMotor(speed);
        SmartDashboard.putNumber("Elevator setpoint", elevatorPID.getSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        amptrapSubsystem.stopAll();
        System.out.println("Amptrap PID ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
