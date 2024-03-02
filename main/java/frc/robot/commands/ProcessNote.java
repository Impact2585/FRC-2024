package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.PivotConstants;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.*;

public class ProcessNote extends CommandBase {
    private final Intake intakeSubsystem;
    private final Indexer indexerSubsystem;

    public ProcessNote(Intake intakeSubsystem, Indexer indexerSubsystem, double seconds) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;

        addRequirements(intakeSubsystem);
        addRequirements(indexerSubsystem);

        intakeSubsystem.spinIn();
        indexerSubsystem.spinIndexer();
        new WaitCommand(seconds);
        intakeSubsystem.stopIntake();
        indexerSubsystem.stopIndex();
    }

    @Override
    public void initialize() {
        System.out.println("ProcessNote started!");
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.stopIntake();
        indexerSubsystem.stopIndex();
        System.out.println("ProcessNote ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
