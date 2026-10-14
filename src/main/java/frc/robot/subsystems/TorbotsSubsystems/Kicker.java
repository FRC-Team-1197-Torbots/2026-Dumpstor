package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.telemetry.Telemetry;
import frc.robot.Constants.ShooterConstants;

/**
 * Kicker Subsystem:
 * Manages the high-speed feed roller located immediately before the main flywheels.
 * 
 * Purpose:
 * When the shooter flywheels are up to speed, the kicker rapidly accelerates
 * the game piece into the drum flywheels to ensure consistent entry velocity
 * and prevent shot trajectory dispersion.
 * 
 * Hardware:
 * - 2 TalonFX motors on CANivore bus "can_s1".
 * - kicker2 follows kicker1 with Opposed motor alignment.
 */
public class Kicker extends SubsystemBase {
    private final TalonFX kicker1; // Master kicker motor
    private final TalonFX kicker2; // Follower kicker motor (opposed)

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Kicker() {
        kicker1 = new TalonFX(ShooterConstants.CANKick1, new CANBus(ShooterConstants.CANLOOP));
        kicker2 = new TalonFX(ShooterConstants.CANKick2, new CANBus(ShooterConstants.CANLOOP));

        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Current limits to prevent brownouts under high acceleration
        commonConfig.CurrentLimits.SupplyCurrentLimit = 40.0; // Amps drawn from the battery
        commonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        commonConfig.CurrentLimits.StatorCurrentLimit = 50.0; // Amps applied to the motor coils
        commonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        kicker1.getConfigurator().apply(commonConfig);
        kicker2.getConfigurator().apply(commonConfig);

        kicker2.setControl(new Follower(kicker1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        Telemetry.log("Kicker/K1 Current", kicker1.getStatorCurrent());
        Telemetry.log("Kicker/K2 Current", kicker2.getStatorCurrent());
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
}