package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IndexerConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Indexer extends SubsystemBase {
    Talon indexerMotor = new Talon(IndexerConstants.indexerPort);

    double indexerStatus = 0.0;

    public Indexer() {

    }

    @Override
    public void periodic() {
        indexerMotor.set(indexerStatus * IndexerConstants.indexerSpeed);
    }

    public void spinIndexer() {
        System.out.println("Indexing...");
        if(indexerStatus != 1.0){
            indexerStatus = 1.0;
        }
        else{
            indexerStatus = 0.0;
        }
    }

    public void reverseIndexer(){
        System.out.println("Indexing halted...");
        if(indexerStatus != -1.0){
            indexerStatus = -1.0;
        }
        else{
            indexerStatus = 0.0;
        }
    }
}
