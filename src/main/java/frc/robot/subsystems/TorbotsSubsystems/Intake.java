package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    public Intake() {
        deploy1 = new TalonFX(IntakeConstants.Intake1, CANBus.systemcore(IntakeConstants.CANLOOP));
        deploy2 = new TalonFX(IntakeConstants.Intake2, CANBus.systemcore(IntakeConstants.CANLOOP));
        roller1 = new TalonFX(IntakeConstants.Roller1, CANBus.systemcore(IntakeConstants.CANLOOP));
        roller2 = new TalonFX(IntakeConstants.Roller2, CANBus.systemcore(IntakeConstants.CANLOOP));

        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Default PID for deploy (Linear / Rack & Pinion style)
        deployConfig.Slot0.kP = 0.0;
        deployConfig.Slot0.kI = 0.0;
        deployConfig.Slot0.kD = 0.0;
        deployConfig.Slot0.kS = 0.0;
        deployConfig.Slot0.kV = 0.0;
        deployConfig.Slot0.kG = 0.0;
        deployConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        deploy1.getConfigurator().apply(deployConfig);
        deploy2.getConfigurator().apply(deployConfig);
        roller1.getConfigurator().apply(rollerConfig);
        roller2.getConfigurator().apply(rollerConfig);

        deploy2.setControl(new Follower(deploy1.getDeviceID(), MotorAlignmentValue.Opposed));
        roller2.setControl(new Follower(roller1.getDeviceID(), MotorAlignmentValue.Opposed));

        // Setup SmartDashboard for live tuning
        SmartDashboard.putNumber("Intake/Tune/kP", 0.0);
        SmartDashboard.putNumber("Intake/Tune/kI", 0.0);
        SmartDashboard.putNumber("Intake/Tune/kD", 0.0);
        SmartDashboard.putNumber("Intake/Tune/kS", 0.0);
        SmartDashboard.putNumber("Intake/Tune/kV", 0.0);
        SmartDashboard.putNumber("Intake/Tune/kG", 0.0);
        
        // Dashboard field for testing roller speeds live
        SmartDashboard.putNumber("Intake/Test/RollerVolts", IntakeConstants.kIntakeRollerVoltage);
    }

    @Override
    public void periodic() {
        // Telemetry for tuning and debugging
        SmartDashboard.putNumber("Intake/Deploy Position (rots)", deploy1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Deploy Velocity (rps)", deploy1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Deploy Applied Volts", deploy1.getMotorVoltage().getValueAsDouble());
    }

    // ==========================================
    // DEPLOY CONTROL
    // ==========================================
    public void setDeployVoltage(double volts) {
        deploy1.setControl(voltageRequest.withOutput(volts));
    }

    public void setDeployPosition(double positionRots) {
        deploy1.setControl(positionRequest.withPosition(positionRots));
    }

    public void stopDeploy() {
        deploy1.stopMotor();
    }

    public Command runDeployCommand(double volts) {
        return this.run(() -> setDeployVoltage(volts)).finallyDo(interrupted -> stopDeploy());
    }

    public Command runDeployToPositionCommand(double positionRots) {
        return this.run(() -> setDeployPosition(positionRots));
    }

    // ==========================================
    // ROLLER CONTROL
    // ==========================================
    public void setRollerVoltage(double volts) {
        roller1.setControl(voltageRequest.withOutput(volts));
    }

    public void stopRollers() {
        roller1.stopMotor();
    }

    public Command runRollersCommand(double volts) {
        return this.run(() -> setRollerVoltage(volts)).finallyDo(interrupted -> stopRollers());
    }

    /**
     * Runs the rollers at a typical FRC intake speed.
     * Often teams aim for a surface speed 2x-3x the robot's drivetrain speed (the "touch it, own it" principle).
     */
    public Command intakeGamePieceCommand() {
        return runRollersCommand(IntakeConstants.kIntakeRollerVoltage);
    }

    public Command ejectGamePieceCommand() {
        return runRollersCommand(IntakeConstants.kEjectRollerVoltage);
    }

    /**
     * Test command that reads voltage from SmartDashboard to easily find the ideal 
     * intake speed without redeploying code.
     */
    public Command testRollerSpeedCommand() {
        return this.run(() -> {
            double testVolts = SmartDashboard.getNumber("Intake/Test/RollerVolts", IntakeConstants.kIntakeRollerVoltage);
            setRollerVoltage(testVolts);
        }).finallyDo(interrupted -> stopRollers());
    }

    public void stopAll() {
        stopDeploy();
        stopRollers();
    }

    // ==========================================
    // UTILITY TUNING COMMANDS & CALCULATIONS
    // ==========================================

    /**
     * Reads Slot0 gains from SmartDashboard and pushes them directly to the master deploy motor.
     */
    public void applyDashboardGains() {
        Slot0Configs slot0 = new Slot0Configs();
        slot0.kP = SmartDashboard.getNumber("Intake/Tune/kP", 0.0);
        slot0.kI = SmartDashboard.getNumber("Intake/Tune/kI", 0.0);
        slot0.kD = SmartDashboard.getNumber("Intake/Tune/kD", 0.0);
        slot0.kS = SmartDashboard.getNumber("Intake/Tune/kS", 0.0);
        slot0.kV = SmartDashboard.getNumber("Intake/Tune/kV", 0.0);
        slot0.kG = SmartDashboard.getNumber("Intake/Tune/kG", 0.0);
        slot0.GravityType = GravityTypeValue.Elevator_Static;

        deploy1.getConfigurator().apply(slot0);
    }

    /**
     * Generates a command to slowly ramp the deploy voltage to find kS (static friction)
     * and kG (gravity for the rack).
     */
    public Command findKSCommand() {
        return this.run(() -> {
            double nextVolt = deploy1.getMotorVoltage().getValueAsDouble() + (0.1 * 0.02); // assumes 20ms loop
            setDeployVoltage(nextVolt);
        }).finallyDo(interrupted -> stopDeploy());
    }
}
