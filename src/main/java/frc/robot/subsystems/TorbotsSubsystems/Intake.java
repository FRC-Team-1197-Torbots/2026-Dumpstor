package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.RobotModeTriggers;
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

/**
 * Intake Subsystem:
 * Manages the robot's ground intake mechanism, which consists of two main mechanisms:
 * 1. Deploy Rack (Linear / Rack & Pinion): Driven by two TalonFX motors (deploy1 & deploy2).
 *    Moves the intake between stowed (0.0 rots) and extended (12.75 rots) positions.
 * 2. Intake Rollers: Driven by two TalonFX motors (roller1 & roller2).
 *    roller2 opposes roller1 to pull game pieces into the hopper.
 */
public class Intake extends SubsystemBase {
    // Hardware motor controllers (Falcon 500 / Kraken X60 on CANivore bus "can_s4")
    private final TalonFX deploy1; // Master deploy motor
    private final TalonFX deploy2; // Follower / paired deploy motor (inverted)
    private final TalonFX roller1; // Master roller motor
    private final TalonFX roller2; // Follower roller motor (opposed)

    // Reusable CTRE Phoenix 6 control requests (avoids object allocation every loop cycle)
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    // Track deployment state (true = extended, false = stowed)
    private boolean isDeployed = false;

    public Intake() {
        deploy1 = new TalonFX(IntakeConstants.Intake1, new CANBus(IntakeConstants.CANLOOP));
        deploy2 = new TalonFX(IntakeConstants.Intake2, new CANBus(IntakeConstants.CANLOOP));
        roller1 = new TalonFX(IntakeConstants.Roller1, new CANBus(IntakeConstants.CANLOOP));
        roller2 = new TalonFX(IntakeConstants.Roller2, new CANBus(IntakeConstants.CANLOOP));

        // ---------------------------------------------------------------------
        // DEPLOY CONFIGURATION
        // ---------------------------------------------------------------------
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        
        // Slot 0 PID for deploy position closed-loop control
        deployConfig.Slot0.kP = IntakeConstants.KP;
        deployConfig.Slot0.kI = 0.0;
        deployConfig.Slot0.kD = 0.0;
        deployConfig.Slot0.kS = 0.0;
        deployConfig.Slot0.kV = 0.0;
        deployConfig.Slot0.kG = 0.0;
        deployConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        // Current limits for deploy:
        // SupplyCurrentLimit (40A) protects the robot battery and PDP/PDH breaker from tripping.
        // StatorCurrentLimit (30A) limits motor torque. If the intake hits the physical hard stop
        // before reaching the exact encoder target, the motors will safely stall without burning
        // out the coils or stripping the rack & pinion teeth.
        deployConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        deployConfig.CurrentLimits.StatorCurrentLimit = 30.0;
        deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // ---------------------------------------------------------------------
        // ROLLER CONFIGURATION
        // ---------------------------------------------------------------------
        TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Roller current limits to protect motors and avoid brownouts under ball friction
        rollerConfig.CurrentLimits.SupplyCurrentLimit = 35.0; // Amps drawn from the battery
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Amps applied to the motor
        rollerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Invert deploy2 relative to deploy1 so both move the rack in the same physical direction
        deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        deploy1.getConfigurator().apply(deployConfig);

        deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        deploy2.getConfigurator().apply(deployConfig);

        roller1.getConfigurator().apply(rollerConfig);
        roller2.getConfigurator().apply(rollerConfig);

        // roller2 is configured as a follower opposed to roller1 to spin inward together
        roller2.setControl(new Follower(roller1.getDeviceID(), MotorAlignmentValue.Opposed));

        // Intake deploy neutral mode triggers:
        // Brake when robot is disabled (keeps intake stowed while carrying or queuing on field).
        // Coast when enabled (allows mechanical compliance / rollback in case of hard robot collision).
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> setDeployNeutralMode(NeutralModeValue.Brake)).ignoringDisable(true)
        );
        RobotModeTriggers.disabled().onFalse(
            Commands.runOnce(() -> setDeployNeutralMode(NeutralModeValue.Coast)).ignoringDisable(true)
        );
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
            setRollerVoltage(IntakeConstants.kIntakeRollerVoltage);
        });
    }

    public Command retractIntakeCommand() {
        return this.runOnce(() -> {
            isDeployed = false;
            setDeployPosition(0.0);
        });
    }

    /**
     * Non-blocking command to agitate the intake deploy back and forth between
     * deployed and halfway positions for a specified duration in seconds.
     * Guaranteed to finish after durationSeconds and return the intake to fully deployed.
     * Perfect for sequential autonomous routines.
     *
     * @param durationSeconds Total time to agitate in seconds
     * @return Autonomous-safe non-blocking Command
     */
    public Command agitateCommand(double durationSeconds) {
        org.wpilib.system.Timer timer = new org.wpilib.system.Timer();
        return this.run(() -> {
            boolean agitateIn = ((int) (timer.get() / IntakeConstants.kAgitateCycleTime)) % 2 == 1;
            if (agitateIn) {
                setDeployPosition(IntakeConstants.kAgitateTargetRots);
            } else {
                setDeployPosition(IntakeConstants.kDeployTargetRots);
            }
        })
        .beforeStarting(timer::restart)
        .finallyDo(interrupted -> setDeployPosition(IntakeConstants.kDeployTargetRots))
        .withTimeout(durationSeconds);
    }

    /**
     * Agitates the intake deploy for the default auto duration (kDefaultAgitateDurationSeconds).
     */
    public Command agitateCommand() {
        return agitateCommand(IntakeConstants.kDefaultAgitateDurationSeconds);
    }

    /**
     * Continuously agitates the intake deploy until interrupted or canceled.
     */
    public Command continuousAgitateCommand() {
        org.wpilib.system.Timer timer = new org.wpilib.system.Timer();
        return this.run(() -> {
            boolean agitateIn = ((int) (timer.get() / IntakeConstants.kAgitateCycleTime)) % 2 == 1;
            if (agitateIn) {
                setDeployPosition(IntakeConstants.kAgitateTargetRots);
            } else {
                setDeployPosition(IntakeConstants.kDeployTargetRots);
            }
        })
        .beforeStarting(timer::restart)
        .finallyDo(interrupted -> setDeployPosition(IntakeConstants.kDeployTargetRots));
    }

    /**
     * Moves the intake halfway in while held, and back out when released.
     * Does NOT require the Intake subsystem so it can be used while shooting.
     */
    public Command operatorAgitateCommand() {
        return org.wpilib.command2.Commands.runEnd(
            () -> setDeployPosition(IntakeConstants.kAgitateTargetRots),
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
            setDeployVoltage(6); // Gently drive forwards into extended hard stop
        }).withTimeout(0.75) // Wait for both sides to hit and stall
        .andThen(() -> {
            stopDeploy();
            deploy1.setPosition(IntakeConstants.kDeployTargetRots);
            deploy2.setPosition(IntakeConstants.kDeployTargetRots);
            isDeployed = true;
        });
    }

    public void setDeployNeutralMode(NeutralModeValue mode) {
        deploy1.configNeutralMode(mode);
        deploy2.configNeutralMode(mode);
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
     * Pulses the intake rollers repeatedly for a fixed duration (safe for autonomous).
     */
    public Command pulseIntakeCommand(double durationSeconds) {
        return pulseIntakeCommand().withTimeout(durationSeconds);
    }

    /**
     * Slowly runs the intake forward in case the piece is slipping and needs 
     * to grab gently without binding up.
     */
    public Command slowIntakeCommand() {
        return runRollersCommand(IntakeConstants.kSlowIntakeVoltage);
    }
}
