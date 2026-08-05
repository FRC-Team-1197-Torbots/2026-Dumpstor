package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Drum extends SubsystemBase {

    private TalonFX Right1, Right2, Left1, Left2;

    public Drum() {
        Right1 = new TalonFX(ShooterConstants.CANDrumRight1);
        Right2 = new TalonFX(ShooterConstants.CANDrumRight2);
        Left1 = new TalonFX(ShooterConstants.CANDrumLeft1);
        Left2 = new TalonFX(ShooterConstants.CANDrumLeft2);
    }

    public void TestDrum() {
        RunDrum(ShooterConstants.TestShootSpeed);
    }

    public void DrumOff() {
        RunDrum(0);
    }

    //Function for testing purposes
    private void RunDrum(float speed) {
        Right1.set(speed);
        Right2.set(speed);

        Left1.set(-speed);
        Left2.set(-speed);
    }
}
