package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Kicker extends SubsystemBase {
    private TalonFX kick1, kick2;
    
    public Kicker() {
        kick1 = new TalonFX(ShooterConstants.CANKick1);
        kick2 = new TalonFX(ShooterConstants.CANKick2);

        kick1.setNeutralMode(NeutralModeValue.Brake);
        kick2.setNeutralMode(NeutralModeValue.Brake);
    }

    public void On() {
        kick1.set(-ShooterConstants.KickSpeed);
        kick2.set(ShooterConstants.KickSpeed);
    }

    public void Off() {
        kick1.set(0);
        kick2.set(0);
    }

    public void TestKick1() { //this motor needs to spin in reverse
        kick1.setNeutralMode(NeutralModeValue.Brake);
        kick2.setNeutralMode(NeutralModeValue.Coast);

        kick1.set(ShooterConstants.KickSpeed);
        kick2.set(0);
    }

    public void TestKick2() {
        kick1.setNeutralMode(NeutralModeValue.Brake);
        kick2.setNeutralMode(NeutralModeValue.Coast);

        kick1.set(0);
        kick2.set(ShooterConstants.KickSpeed);
    }

}
