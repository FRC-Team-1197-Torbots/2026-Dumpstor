package frc.robot.subsystems.TorbotsSubsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import org.wpilib.command2.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Drum extends SubsystemBase {

    private TalonFX Right1, Right2, Left1, Left2;

    public Drum() {
        Right1 = new TalonFX(ShooterConstants.CANDrumRight1, CANBus.systemcore(0));
        //Right2 = new TalonFX(ShooterConstants.CANDrumRight2);
        //Left1 = new TalonFX(ShooterConstants.CANDrumLeft1);
        //Left2 = new TalonFX(ShooterConstants.CANDrumLeft2);
    }

    public void DrumOff() {
        RunDrum(0);
    }

    //Function for testing purposes
    private void RunDrum(float speed) {
        // Right1.set(speed);
        // Right2.set(speed);

        // Left1.set(-speed);
        // Left2.set(-speed);
    }
}
