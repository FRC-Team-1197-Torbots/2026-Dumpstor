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

public class Drum extends SubsystemBase {

    private final TalonFX Right1; // Single Master
    private final TalonFX Right2;
    private final TalonFX Left1;
    private final TalonFX Left2;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private double testKS = 0.0f;

    public Drum() {
        Right1 = new TalonFX(ShooterConstants.CANDrumRight1, CANBus.systemcore(ShooterConstants.CANLOOP));
        Right2 = new TalonFX(ShooterConstants.CANDrumRight2, CANBus.systemcore(ShooterConstants.CANLOOP));
        Left1 = new TalonFX(ShooterConstants.CANDrumLeft1, CANBus.systemcore(ShooterConstants.CANLOOP));
        Left2 = new TalonFX(ShooterConstants.CANDrumLeft2, CANBus.systemcore(ShooterConstants.CANLOOP));

        // General configuration applied to all 4 motors
        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Slot 0 default gains on the master
        commonConfig.Slot0.kS = 4.25;
        commonConfig.Slot0.kV = 0.0;
        commonConfig.Slot0.kP = 0.9;
        commonConfig.Slot0.kD = 0.0;

        // Flywheels take massive energy to spin up quickly. With 4 motors, a full unrestrained
        // spin-up would draw >200A and cause a brownout. We cap the supply limit to 35-40A per motor
        // (140-160A total) which a healthy battery can handle for the 1-2 second spin-up.
        commonConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        commonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // Flywheels rarely stall, but if a game piece gets completely jammed, this protects the motors.
        commonConfig.CurrentLimits.StatorCurrentLimit = 60.0;
        commonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        Right1.getConfigurator().apply(commonConfig);
        Right2.getConfigurator().apply(commonConfig);
        Left1.getConfigurator().apply(commonConfig);
        Left2.getConfigurator().apply(commonConfig);

        // Configure followers to track Right1 (ID passed dynamically)
        Right2.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Aligned));
        Left1.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Opposed));
        Left2.setControl(new Follower(Right1.getDeviceID(), MotorAlignmentValue.Opposed));

        // Telemetry entries for live tuning
        Telemetry.putNumber("Drum/Tune/kS", 0.0);
        Telemetry.putNumber("Drum/Tune/kV", 0.0);
        Telemetry.putNumber("Drum/Tune/kP", 0.0);
        Telemetry.putNumber("Drum/Tune/kD", 0.0);
    }

    @Override
    public void periodic() {
        Telemetry.putNumber("Drum/Velocity RPS", getVelocityRPS());
        Telemetry.putNumber("Drum/Applied Volts", getMotorVoltage());
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
     * Note the voltage on Telemetry the moment the drum begins rolling.
     */
    public Command findKSCommand() {
        return this.run(() -> {
            
            double nextVolt = testKS + 0.001d;
            setVoltage(nextVolt);
            testKS = nextVolt;
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
}
