package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
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

    private boolean isDeployed = false;

    public Intake() {
        deploy1 = new TalonFX(IntakeConstants.Intake1, new CANBus(IntakeConstants.CANLOOP));
        deploy2 = new TalonFX(IntakeConstants.Intake2, new CANBus(IntakeConstants.CANLOOP));
        roller1 = new TalonFX(IntakeConstants.Roller1, new CANBus(IntakeConstants.CANLOOP));
        roller2 = new TalonFX(IntakeConstants.Roller2, new CANBus(IntakeConstants.CANLOOP));

        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Default PID for deploy (Linear / Rack & Pinion style)
        deployConfig.Slot0.kP = IntakeConstants.KP;
        deployConfig.Slot0.kI = 0.0;
        deployConfig.Slot0.kD = 0.0;
        deployConfig.Slot0.kS = 0.0;
        deployConfig.Slot0.kV = 0.0;
        deployConfig.Slot0.kG = 0.0;
        deployConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Current limits for deploy to prevent breaking hard stops
        deployConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Basic current limits to prevent brownouts
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 35.0; // Amps drawn from the battery
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Amps applied to the motor
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        deploy1.getConfigurator().apply(deployConfig);

        // Invert deploy2 so they can be driven with the same position request
        deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deploy2.getConfigurator().apply(deployConfig);

        roller1.getConfigurator().apply(rollerConfig);
        roller2.getConfigurator().apply(rollerConfig);

        // roller2 still follows roller1
        roller2.setControl(new Follower(roller1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        // Telemetry for tuning and debugging
        // Telemetry.log("Intake/Deploy Position (rots)", deploy1.getPosition().getValueAsDouble());
    }

    // ==========================================
    // DEPLOY CONTROL
    // ==========================================
    public double getDeployPosition() {
        return deploy1.getPosition().getValueAsDouble();
    }

    public void setDeployVoltage(double volts) {
        deploy1.setControl(voltageRequest.withOutput(volts));
        deploy2.setControl(voltageRequest.withOutput(volts));
    }

    public void setDeployPosition(double positionRots) {
        deploy1.setControl(positionRequest.withPosition(positionRots));
        deploy2.setControl(positionRequest.withPosition(positionRots));
    }

    public void stopDeploy() {
        deploy1.stopMotor();
        deploy2.stopMotor();
    }

    public Command runDeployCommand(double volts) {
        return this.run(() -> setDeployVoltage(volts)).finallyDo(interrupted -> stopDeploy());
    }

    public Command runDeployToPositionCommand(double positionRots) {
        return this.run(() -> setDeployPosition(positionRots));
    }

    

    /**
     * Toggles the intake deploy between stowed (0.0 rots) and deployed (kDeployTargetRots).
     * Since we configured Stator Current Limits (30A), if the intake hits the hard stops 
     * before reaching the exact rotations, the motor will safely stall and hold position 
     * without burning out or breaking the mechanism.
     */
    public Command toggleDeployCommand() {
        return this.runOnce(() -> {
            isDeployed = !isDeployed;
            if (isDeployed) {
                setDeployPosition(IntakeConstants.kDeployTargetRots);
            } else {
                setDeployPosition(0.0);
            }
        });
    }

    public Command deployIntakeCommand() {
        return this.runOnce(() -> {
            isDeployed = true;
            setDeployPosition(IntakeConstants.kDeployTargetRots);
        });
    }

    public Command retractIntakeCommand() {
        return this.runOnce(() -> {
            isDeployed = false;
            setDeployPosition(0.0);
        });
    }

    /**
     * Moves the intake halfway in while held, and back out when released.
     * Does NOT require the Intake subsystem so it can be used while shooting.
     */
    public Command operatorAgitateCommand() {
        return org.wpilib.command2.Commands.runEnd(
            () -> setDeployPosition(IntakeConstants.kDeployTargetRots / 2.0),
            () -> setDeployPosition(IntakeConstants.kDeployTargetRots)
        );
    }

    public void zeroDeployEncoder() {
        deploy1.setPosition(0.0);
        deploy2.setPosition(0.0);
    }

    /**
     * Drives both sides of the intake gently into the extended hard stop 
     * to square the mechanism, then sets the encoders to the max extension.
     */
    public Command squareIntakeCommand() {
        return this.run(() -> {
            setDeployVoltage(2.5); // Gently drive forwards into extended hard stop
        }).withTimeout(0.75) // Wait for both sides to hit and stall
        .andThen(() -> {
            stopDeploy();
            deploy1.setPosition(IntakeConstants.kDeployTargetRots);
            deploy2.setPosition(IntakeConstants.kDeployTargetRots);
            isDeployed = true;
        });
    }

    public void setDeployNeutralMode(NeutralModeValue mode) {
        com.ctre.phoenix6.configs.MotorOutputConfigs motorOutput = new com.ctre.phoenix6.configs.MotorOutputConfigs();
        
        deploy1.getConfigurator().refresh(motorOutput);
        motorOutput.NeutralMode = mode;
        deploy1.getConfigurator().apply(motorOutput);

        deploy2.getConfigurator().refresh(motorOutput);
        motorOutput.NeutralMode = mode;
        deploy2.getConfigurator().apply(motorOutput);
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
     * Pulses the intake rollers rapidly between reverse and off to shake loose 
     * a jammed piece without fully ejecting it out of the robot's area.
     */
    public Command pulseIntakeCommand() {
        return org.wpilib.command2.Commands.sequence(
            runRollersCommand(IntakeConstants.kPulseReverseVoltage).withTimeout(0.2),
            org.wpilib.command2.Commands.waitSeconds(0.1),
            runRollersCommand(IntakeConstants.kPulseReverseVoltage).withTimeout(0.2),
            org.wpilib.command2.Commands.waitSeconds(0.1)
        ).repeatedly();
    }

    /**
     * Slowly runs the intake forward in case the piece is slipping and needs 
     * to grab gently without binding up.
     */
    public Command slowIntakeCommand() {
        return runRollersCommand(IntakeConstants.kSlowIntakeVoltage);
    }
}
