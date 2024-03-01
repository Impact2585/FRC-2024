package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.ClimbConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SubsystemBase {
    Talon leftClimber = new Talon(ClimbConstants.leftClimberPort);
    Talon rightClimber = new Talon(ClimbConstants.rightClimberPort);

    public Climb() {

    }

    @Override
    public void periodic() {

    }

    public void deployArms() {
        System.out.println("Lifting arms...");
        leftClimber.set(ClimbConstants.climbSpeed);
        rightClimber.set(ClimbConstants.climbSpeed);
    }

    public void pullUp(){
        System.out.println("Pulling up...");
        leftClimber.set(-ClimbConstants.climbSpeed);
        rightClimber.set(-ClimbConstants.climbSpeed);
    }

    public void stopClimb(){
        leftClimber.set(0);
        rightClimber.set(0);
    }
}
