package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

public class Kicker extends SubsystemBase {
    private final TalonFX kicker1;
    private final TalonFX kicker2;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Kicker() {
        kicker1 = new TalonFX(ShooterConstants.CANKick1, CANBus.systemcore(ShooterConstants.CANLOOP));
        kicker2 = new TalonFX(ShooterConstants.CANKick2, CANBus.systemcore(ShooterConstants.CANLOOP));

        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kicker1.getConfigurator().apply(commonConfig);
        kicker2.getConfigurator().apply(commonConfig);

        kicker2.setControl(new Follower(kicker1.getDeviceID(), MotorAlignmentValue.Opposed));

        // Dashboard field for testing kicker feeding speeds live
        SmartDashboard.putNumber("Shooter/Kicker/TestVolts", ShooterConstants.kKickerFeedVoltage);
    }

    public void setVoltage(double volts) {
        kicker1.setControl(voltageRequest.withOutput(volts));
    }

    public void stop() {
        kicker1.stopMotor();
    }

    public Command runVoltageCommand(double volts) {
        return this.run(() -> setVoltage(volts)).finallyDo(interrupted -> stop());
    }

    /**
     * Feeds game pieces into the main shooter flywheels.
     * This usually needs to be fast (8-12V) so the game piece doesn't lose momentum.
     */
    public Command feedShooterCommand() {
        return runVoltageCommand(ShooterConstants.kKickerFeedVoltage);
    }

    /**
     * Reverses the kicker to unjam game pieces.
     */
    public Command unjamCommand() {
        return runVoltageCommand(ShooterConstants.kKickerUnjamVoltage);
    }

    /**
     * Test command that reads voltage from SmartDashboard to tune 
     * kicker feed speed live.
     */
    public Command testKickerSpeedCommand() {
        return this.run(() -> {
            double testVolts = SmartDashboard.getNumber("Shooter/Kicker/TestVolts", ShooterConstants.kKickerFeedVoltage);
            setVoltage(testVolts);
        }).finallyDo(interrupted -> stop());
    }
}
