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
import frc.robot.Constants.IntakeConstants;

// Subsystem will use the two deploy motors to extend the intake and then power the rollers to intake balls
public class Intake extends SubsystemBase {
    private final TalonFX deploy1;
    private final TalonFX deploy2;
    private final TalonFX roller1;
    private final TalonFX roller2;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Intake() {
        deploy1 = new TalonFX(IntakeConstants.Intake1, CANBus.systemcore(IntakeConstants.CANLOOP));
        deploy2 = new TalonFX(IntakeConstants.Intake2, CANBus.systemcore(IntakeConstants.CANLOOP));
        roller1 = new TalonFX(IntakeConstants.Roller1, CANBus.systemcore(IntakeConstants.CANLOOP));
        roller2 = new TalonFX(IntakeConstants.Roller2, CANBus.systemcore(IntakeConstants.CANLOOP));

        TalonFXConfiguration brakeConfig = new TalonFXConfiguration();
        brakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        TalonFXConfiguration coastConfig = new TalonFXConfiguration();
        coastConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        deploy1.getConfigurator().apply(brakeConfig);
        deploy2.getConfigurator().apply(brakeConfig);
        roller1.getConfigurator().apply(coastConfig);
        roller2.getConfigurator().apply(coastConfig);

        deploy2.setControl(new Follower(deploy1.getDeviceID(), MotorAlignmentValue.Opposed));
        roller2.setControl(new Follower(roller1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setDeployVoltage(double volts) {
        deploy1.setControl(voltageRequest.withOutput(volts));
    }

    public void setRollerVoltage(double volts) {
        roller1.setControl(voltageRequest.withOutput(volts));
    }

    public void stopDeploy() {
        deploy1.stopMotor();
    }

    public void stopRollers() {
        roller1.stopMotor();
    }

    public void stopAll() {
        stopDeploy();
        stopRollers();
    }

    public Command runRollersCommand(double volts) {
        return this.run(() -> setRollerVoltage(volts)).finallyDo(interrupted -> stopRollers());
    }

    public Command runDeployCommand(double volts) {
        return this.run(() -> setDeployVoltage(volts)).finallyDo(interrupted -> stopDeploy());
    }
}
