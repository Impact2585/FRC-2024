package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Indexer extends SubsystemBase {
    Talon indexerMotor = new Talon(IndexerConstants.indexerPort);

    public Indexer() {
        this.stopIndex();
    }

    @Override
    public void periodic() {
    }

    public void spinIndexer() {
        indexerMotor.set(IndexerConstants.indexerSpeed);
    }

    public void reverseIndexer(){
        indexerMotor.set(-IndexerConstants.indexerSpeed);
    }

    public void stopIndex(){
        indexerMotor.set(0);
    }
}
