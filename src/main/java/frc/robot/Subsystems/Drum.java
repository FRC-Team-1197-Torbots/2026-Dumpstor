package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Drum extends SubsystemBase {

    private TalonFX Right1, Right2, Left1, Left2;

    private MotionMagicVelocityVoltage m_request = new MotionMagicVelocityVoltage(0);
    // private MotionMagicVelocityVoltage m_requestRun = new MotionMagicVelocityVoltage(2000);

    public Drum() {
        Right1 = new TalonFX(ShooterConstants.CANDrumRight1);
        Right2 = new TalonFX(ShooterConstants.CANDrumRight2);
        Left1 = new TalonFX(ShooterConstants.CANDrumLeft1);
        Left2 = new TalonFX(ShooterConstants.CANDrumLeft2);

        Right2.setControl(new Follower(1, MotorAlignmentValue.Aligned));
        Left1.setControl(new Follower(1, MotorAlignmentValue.Opposed));
        Left2.setControl(new Follower(1, MotorAlignmentValue.Opposed));
    }

    public void TestDrum() {
        Right1.setControl(m_request.withVelocity(2000));
    }

    public void DrumOff() {
        // RunDrum(0);
        Right1.setControl(m_request.withVelocity(0));
    }

    //Function for testing purposes
    private void RunDrum(float speed) {
        Right1.setControl(m_request.withVelocity(2000));
        // Right2.set(speed);

        // Left1.set(-speed);
        // Left2.set(-speed);
    }
}
