package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.telemetry.Telemetry;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

/**
 * Drum Subsystem:
 * Manages the high-speed shooter flywheel mechanism.
 * 
 * Hardware Layout:
 * - 4 Falcon 500 / Kraken X60 motors on CANivore bus "can_s1".
 * - Right1 acts as the single Master motor.
 * - Right2 follows Right1 (Aligned direction).
 * - Left1 and Left2 follow Right1 (Opposed direction, as they mirror the shaft on the other side).
 * 
 * Control Mode:
 * - CTRE Phoenix 6 VelocityVoltage closed-loop control running on-motor at 1kHz.
 * - Units: Rotations Per Second (RPS). Note: RPM = RPS * 60.0.
 */
public class Drum extends SubsystemBase {

    private final TalonFX Right1; // Single Master
    private final TalonFX Right2; // Follower
    private final TalonFX Left1;  // Follower (opposed)
    private final TalonFX Left2;  // Follower (opposed)

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Drum() {
        Right1 = new TalonFX(ShooterConstants.CANDrumRight1, new CANBus(ShooterConstants.CANLOOP));
        Right2 = new TalonFX(ShooterConstants.CANDrumRight2, new CANBus(ShooterConstants.CANLOOP));
        Left1 = new TalonFX(ShooterConstants.CANDrumLeft1, new CANBus(ShooterConstants.CANLOOP));
        Left2 = new TalonFX(ShooterConstants.CANDrumLeft2, new CANBus(ShooterConstants.CANLOOP));

        // General configuration applied to all 4 motors
        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Slot 0 velocity PID gains on the master motor:
        // kS: Static friction feedforward (volts added when velocity is commanded)
        // kV: Velocity feedforward (volts per RPS)
        // kP: Proportional gain (volts per unit of RPS error)
        commonConfig.Slot0.kS = 7.5;
        commonConfig.Slot0.kV = 0.0;
        commonConfig.Slot0.kP = 0.7;
        commonConfig.Slot0.kD = 0.0;

        // Flywheels take massive energy to spin up quickly. With 4 motors, an unrestricted
        // spin-up would draw >200A and cause a brownout. We cap the supply limit to 40A per motor
        // (160A total) which a healthy battery can handle during the spin-up ramp.
        commonConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        commonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Stator Current Limit: Limits torque and protects the motor coils if a ball gets jammed.
        commonConfig.CurrentLimits.StatorCurrentLimit = 50.0;
        commonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        Right1.getConfigurator().apply(commonConfig);
        Right2.getConfigurator().apply(commonConfig);
        Left1.getConfigurator().apply(commonConfig);
        Left2.getConfigurator().apply(commonConfig);

        // Configure followers to track Right1 master:
        // Right2 spins the same direction as Right1.
        // Left1 and Left2 are mounted symmetrically facing the opposite way, so they are Opposed.
        Right2.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Aligned));
        Left1.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Opposed));
        Left2.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        Telemetry.log("Drum/Velocity RPS", getVelocityRPS());
        Telemetry.log("Drum/R1", Right1.getStatorCurrent().getValue());
        Telemetry.log("Drum/R2", Right2.getStatorCurrent().getValue());
        Telemetry.log("Drum/L1", Left1.getStatorCurrent().getValue());
        Telemetry.log("Drum/L2", Left2.getStatorCurrent().getValue());
    }

    /** Directly commands output voltage to the master motor. */
    public void setVoltage(double volts) {
        Right1.setControl(voltageRequest.withOutput(volts));
    }

    /** Commands closed-loop velocity in Rotations Per Second (RPS). */
    public void setVelocityRPS(double targetRPS) {
        Right1.setControl(velocityRequest.withVelocity(targetRPS));
    }

    public double getVelocityRPS() {
        return Right1.getVelocity().getValueAsDouble();
    }

    public double getMotorVoltage() {
        return Right1.getMotorVoltage().getValueAsDouble();
    }

    public void stop() {
        Right1.stopMotor();
    }
}
