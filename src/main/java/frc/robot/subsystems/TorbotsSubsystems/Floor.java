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

public class Floor extends SubsystemBase {
    private final TalonFX floor1;
    private final TalonFX floor2;   

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Floor() {
        floor1 = new TalonFX(ShooterConstants.CANFloor1, CANBus.systemcore(ShooterConstants.CANLOOP));
        floor2 = new TalonFX(ShooterConstants.CANFloor2, CANBus.systemcore(ShooterConstants.CANLOOP));

        TalonFXConfiguration commonConfig = new TalonFXConfiguration();
        commonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
}
