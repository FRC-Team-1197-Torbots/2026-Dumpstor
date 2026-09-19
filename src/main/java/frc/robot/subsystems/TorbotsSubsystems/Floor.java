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

public class Floor extends SubsystemBase {
    private final TalonFX floor1;
    private final TalonFX floor2;   

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Floor() {
        floor1 = new TalonFX(ShooterConstants.CANFloor1, CANBus.systemcore(ShooterConstants.CANLOOP));
        floor2 = new TalonFX(ShooterConstants.CANFloor2, CANBus.systemcore(ShooterConstants.CANLOOP));

        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Basic current limits to prevent brownouts
        commonConfig.CurrentLimits.SupplyCurrentLimit = 35.0; // Amps drawn from the battery
        commonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        commonConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Amps applied to the motor
        commonConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        floor1.getConfigurator().apply(commonConfig);
        floor2.getConfigurator().apply(commonConfig);

        floor2.setControl(new Follower(floor1.getDeviceID(), MotorAlignmentValue.Opposed));

        // Dashboard field for testing floor feeding speeds live
        SmartDashboard.putNumber("Shooter/Floor/TestVolts", ShooterConstants.kFloorFeedVoltage);
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
     * Feeds game pieces into the shooter/kicker.
     */
    public Command feedShooterCommand() {
        return runVoltageCommand(ShooterConstants.kFloorFeedVoltage);
    }

    /**
     * Reverses the floor to unjam game pieces.
     */
    public Command unjamCommand() {
        return runVoltageCommand(ShooterConstants.kFloorUnjamVoltage);
    }

    /**
     * Test command that reads voltage from SmartDashboard to tune 
     * floor feed speed live.
     */
    public Command testFloorSpeedCommand() {
        return this.run(() -> {
            double testVolts = SmartDashboard.getNumber("Shooter/Floor/TestVolts", ShooterConstants.kFloorFeedVoltage);
            setVoltage(testVolts);
        }).finallyDo(interrupted -> stop());
    }
}
