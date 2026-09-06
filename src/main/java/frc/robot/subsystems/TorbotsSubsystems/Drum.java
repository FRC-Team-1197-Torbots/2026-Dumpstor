package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.ShooterConstants;

public class Drum extends SubsystemBase {

    private final TalonFX Right1; // Single Master
    private final TalonFX Right2;
    private final TalonFX Left1;
    private final TalonFX Left2;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public Drum() {
        Right1 = new TalonFX(ShooterConstants.CANDrumRight1, CANBus.systemcore(ShooterConstants.CANLOOP));
        Right2 = new TalonFX(ShooterConstants.CANDrumRight2, CANBus.systemcore(ShooterConstants.CANLOOP));
        Left1 = new TalonFX(ShooterConstants.CANDrumRight1, CANBus.systemcore(ShooterConstants.CANLOOP));
        Left2 = new TalonFX(ShooterConstants.CANDrumRight2, CANBus.systemcore(ShooterConstants.CANLOOP));

        // General configuration applied to all 4 motors
        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Slot 0 default gains on the master
        commonConfig.Slot0.kS = 0.0;
        commonConfig.Slot0.kV = 0.0;
        commonConfig.Slot0.kP = 0.0;
        commonConfig.Slot0.kD = 0.0;

        Right1.getConfigurator().apply(commonConfig);
        Right2.getConfigurator().apply(commonConfig);
        Left1.getConfigurator().apply(commonConfig);
        Left2.getConfigurator().apply(commonConfig);

        // Configure followers to track Right1 (ID passed dynamically)
        Right2.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Aligned));
        Left1.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Opposed));
        Left2.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Opposed));

        // SmartDashboard entries for live tuning
        SmartDashboard.putNumber("Drum/Tune/kS", 0.0);
        SmartDashboard.putNumber("Drum/Tune/kV", 0.0);
        SmartDashboard.putNumber("Drum/Tune/kP", 0.0);
        SmartDashboard.putNumber("Drum/Tune/kD", 0.0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Drum/Velocity RPS", getVelocityRPS());
        SmartDashboard.putNumber("Drum/Applied Volts", getMotorVoltage());
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

    // ==========================================
    // UTILITY TUNING COMMANDS & CALCULATIONS
    // ==========================================

    /**
     * Slowly ramps open-loop voltage by 0.1V/sec.
     * Note the voltage on SmartDashboard the moment the drum begins rolling.
     */
    public Command findKSCommand() {
        return this.run(() -> {
            double nextVolt = Right1.getMotorVoltage().getValueAsDouble() + (0.1 * 0.02);
            setVoltage(nextVolt);
        }).finallyDo(interrupted -> stop());
    }

    /**
     * Runs drum at a target test voltage (e.g., 8.0V - 10.0V).
     * Once drum hits top speed, pass the steady-state RPS to computeKV().
     */
    public Command runVoltageCharacterization(double testVolts) {
        return this.run(() -> setVoltage(testVolts))
                .finallyDo(interrupted -> stop());
    }

    /**
     * Calculates kV in Volts per RPS.
     * Formula: kV = (AppliedVolts - kS) / MeasuredRPS
     */
    public static double computeKV(double appliedVolts, double steadyStateRPS, double measuredKS) {
        if (steadyStateRPS <= 0.01)
            return 0.0;
        return (appliedVolts - measuredKS) / steadyStateRPS;
    }

    /**
     * Reads Slot0 gains from SmartDashboard and pushes them directly to the master
     * motor.
     */
    public void applyDashboardGains() {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kS = SmartDashboard.getNumber("Drum/Tune/kS", 0.0);
        slot0.kV = SmartDashboard.getNumber("Drum/Tune/kV", 0.0);
        slot0.kP = SmartDashboard.getNumber("Drum/Tune/kP", 0.0);
        slot0.kD = SmartDashboard.getNumber("Drum/Tune/kD", 0.0);

        Right1.getConfigurator().apply(slot0);
    }
}
