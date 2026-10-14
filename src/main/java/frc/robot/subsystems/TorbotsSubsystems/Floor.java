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
import frc.robot.Constants.ShooterConstants;
import org.wpilib.telemetry.Telemetry;

/**
 * Floor Subsystem:
 * Manages the hopper floor conveyor belts/rollers.
 * 
 * Hardware:
 * - 2 TalonFX motors on CANivore bus "can_s1".
 * - floor2 follows floor1 with Opposed motor alignment.
 * 
 * Note:
 * In the current competition configuration, game pieces are fed into the shooter
 * using the intake rollers and kicker directly. The floor subsystem remains available
 * if floor belt feeding is reintroduced.
 */
public class Floor extends SubsystemBase {
    private final TalonFX floor1; // Master floor motor
    private final TalonFX floor2; // Follower floor motor (opposed)

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Floor() {
        floor1 = new TalonFX(ShooterConstants.CANFloor1, new CANBus(ShooterConstants.CANLOOP));
        floor2 = new TalonFX(ShooterConstants.CANFloor2, new CANBus(ShooterConstants.CANLOOP));

        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Current limits (20A supply, 20A stator) to prevent belt binding damage
        commonConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
        commonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        commonConfig.CurrentLimits.StatorCurrentLimit = 20.0;
        commonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        floor1.getConfigurator().apply(commonConfig);
        floor2.getConfigurator().apply(commonConfig);

        floor2.setControl(new Follower(floor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setVoltage(double volts) {
        floor1.setControl(voltageRequest.withOutput(volts));
    }

    public void stop() {
        floor1.stopMotor();
    }

    public Command runVoltageCommand(double volts) {
        return this.run(() -> setVoltage(volts)).finallyDo(interrupted -> stop());
    }

    /**
     * Feeds game pieces forward through the floor into the kicker/shooter.
     */
    public Command feedShooterCommand() {
        return runVoltageCommand(ShooterConstants.kFloorFeedVoltage);
    }

    /**
     * Reverses the floor conveyor to unjam game pieces.
     */
    public Command unjamCommand() {
        return runVoltageCommand(ShooterConstants.kFloorUnjamVoltage);
    }

    @Override
    public void periodic() {
        Telemetry.log("Floor/F1", floor1.getStatorCurrent());
        Telemetry.log("Floor/F2", floor2.getStatorCurrent());
    }
}
