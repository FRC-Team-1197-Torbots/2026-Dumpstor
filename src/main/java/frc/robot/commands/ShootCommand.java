package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TorbotsSubsystems.Drum;
import frc.robot.subsystems.TorbotsSubsystems.Intake;
import frc.robot.subsystems.TorbotsSubsystems.Kicker;

/**
 * ShootCommand coordinates the full shooter sequence:
 * 1. Accelerates the main drum flywheels to target speed (Rotations Per Second).
 * 2. Monitors drum velocity until it reaches target speed within tolerance (+/- 2.0 RPS)
 *    OR until a fallback safety timer (kSpinupTimeoutSeconds) expires.
 * 3. Once at speed ("isShooting = true"), powers the kicker and intake rollers to feed
 *    game pieces into the flywheels.
 * 4. In Autonomous: Optionally oscillates the intake deploy halfway (agitation) to prevent
 *    game pieces from jamming, and cleanly terminates after shootDurationSeconds.
 * 5. In Teleop: Runs continuously as long as the trigger/button is held down.
 */
public class ShootCommand extends Command {
    // Subsystem dependencies
    private final Drum drum;       // Main flywheels
    private final Kicker kicker;   // High-speed accelerator roller
    private final Intake intake;   // Intake deploy rack & intake rollers

    // Target velocity in Rotations Per Second (RPS = RPM / 60.0)
    private final double targetRPS;

    // Autonomous shoot duration in seconds. If <= 0, the command runs indefinitely (teleop mode).
    private final double shootDurationSeconds;

    // Flag whether to oscillate the intake deploy back and forth while feeding
    private final boolean agitateIntake;

    // Tolerance in RPS. Flywheel speed fluctuates when balls enter; 2.0 RPS is a healthy balance
    // between shot velocity accuracy and quick consecutive feeding.
    private static final double RPS_TOLERANCE = 2.0;

    // State machine flags & timers
    private boolean isShooting = false;                // Latches true once drum is up to speed
    private final Timer spinupTimer = new Timer();         // Tracks time spent spinning up
    private final Timer shootDurationTimer = new Timer();  // Tracks time actively feeding pieces

    /**
     * Teleoperated Constructor:
     * Continuous shooting that finishes when the driver releases the gamepad trigger.
     * Intake agitation is set to false here so the operator has exclusive control over
     * manual agitation using operatorJoystick.
     *
     * @param drum      The drum flywheel subsystem
     * @param kicker    The kicker roller subsystem
     * @param intake    The intake subsystem
     * @param targetRPS Target flywheel speed in Rotations Per Second (RPS)
     */
    public ShootCommand(Drum drum, Kicker kicker, Intake intake, double targetRPS) {
        this(drum, kicker, intake, targetRPS, -1.0, false);
    }

    /**
     * Autonomous Constructor:
     * Feeds game pieces for a set duration after reaching target speed, then automatically finishes.
     *
     * @param drum                 The drum flywheel subsystem
     * @param kicker               The kicker roller subsystem
     * @param intake               The intake subsystem
     * @param targetRPS            Target flywheel speed in Rotations Per Second (RPS)
     * @param shootDurationSeconds Number of seconds to feed kicker/rollers after reaching speed.
     *                             Set <= 0 for continuous shooting (teleop).
     * @param agitateIntake        Whether to oscillate the intake deploy while shooting to unjam pieces.
     */
    public ShootCommand(
        Drum drum,
        Kicker kicker,
        Intake intake,
        double targetRPS,
        double shootDurationSeconds,
        boolean agitateIntake
    ) {
        this.drum = drum;
        this.kicker = kicker;
        this.intake = intake;
        this.targetRPS = targetRPS;
        this.shootDurationSeconds = shootDurationSeconds;
        this.agitateIntake = agitateIntake;

        // Subsystem requirements: Tells WPILib's CommandScheduler that this command
        // requires exclusive access to these subsystems.
        addRequirements(drum, kicker, intake);
    }

    /**
     * Called once when the command is scheduled.
     * Resets timers, spins up flywheels, and stops feed mechanisms until speed is reached.
     */
    @Override
    public void initialize() {
        isShooting = false;
        spinupTimer.restart();
        shootDurationTimer.reset();

        // Spin up the flywheels to the target RPS
        drum.setVelocityRPS(targetRPS);

        // Ensure all feeder mechanisms are stopped while spinning up
        kicker.stop();
        intake.stopRollers();

        // Ensure intake is fully extended to receive balls
        intake.setDeployPosition(IntakeConstants.kDeployTargetRots);
    }

    /**
     * Called every 20ms scheduler cycle while the command is running.
     */
    @Override
    public void execute() {
        org.wpilib.telemetry.Telemetry.log("Auto/isShooting", isShooting);
        org.wpilib.telemetry.Telemetry.log("Auto/TargetRPS", targetRPS);
        org.wpilib.telemetry.Telemetry.log("Shooter/VelocityRPS", drum.getVelocityRPS());

        // Step 1: Check if the flywheels have reached shooting velocity.
        // We latch isShooting to true if:
        // A) Current velocity is within RPS_TOLERANCE of targetRPS, OR
        // B) kSpinupTimeoutSeconds has elapsed (prevents battery sag / slight friction from halting auto).
        if (!isShooting) {
            boolean atSpeed = Math.abs(drum.getVelocityRPS() - targetRPS) <= RPS_TOLERANCE;
            boolean spinupTimeout = spinupTimer.hasElapsed(ShooterConstants.kSpinupTimeoutSeconds);

            if (atSpeed || spinupTimeout) {
                isShooting = true;
                shootDurationTimer.restart(); // Start timing active ball feeding
            }
        }

        // Step 2: Feed game pieces once up to speed
        if (isShooting) {
            // Power the kicker to fling balls into flywheels
            kicker.setVoltage(ShooterConstants.kKickerFeedVoltage);

            // Power intake rollers to push balls back towards kicker
            intake.setRollerVoltage(IntakeConstants.kIntakeRollerVoltage);

            // Autonomous Agitation:
            // Uses a simple time-modulo oscillation (every 0.3s) to pulse the intake deploy
            // halfway in and back out, breaking any ball bridging/jamming inside the hopper.
            if (agitateIntake) {
                double timeSinceFeed = shootDurationTimer.get();
                boolean agitateHalfway = ((int) (timeSinceFeed / IntakeConstants.kAgitateCycleTime)) % 2 == 1;
                if (agitateHalfway) {
                    intake.setDeployPosition(IntakeConstants.kAgitateTargetRots);
                } else {
                    intake.setDeployPosition(IntakeConstants.kDeployTargetRots);
                }
            }
        }
    }

    /**
     * Called when the command ends (either naturally via isFinished() or interrupted by another command/button release).
     * Returns the robot to a safe, neutral state.
     */
    @Override
    public void end(boolean interrupted) {
        // Return drum flywheels to idle speed (preserves battery and motor heat while ready for next shot)
        drum.setVelocityRPS(ShooterConstants.IdleShooterSpeed);

        // Immediately stop feed mechanisms
        kicker.stop();
        intake.stopRollers();

        // Return intake to fully deployed position
        intake.setDeployPosition(IntakeConstants.kDeployTargetRots);
    }

    /**
     * Evaluates whether the command should finish.
     * - In Autonomous (shootDurationSeconds > 0): Finishes when active feeding time completes
     *   or when the overall safety timeout expires.
     * - In Teleop (shootDurationSeconds <= 0): Returns false; runs as long as the driver holds the trigger.
     */
    @Override
    public boolean isFinished() {
        if (shootDurationSeconds > 0) {
            // Has the active feeding duration finished?
            boolean durationElapsed = isShooting && shootDurationTimer.hasElapsed(shootDurationSeconds);

            // Fallback safety timeout (total spin-up timeout + feed duration + 0.5s margin)
            // Ensures an auto sequence NEVER hangs forever, even if something physically stalls
            boolean safetyTimeout = spinupTimer.hasElapsed(ShooterConstants.kSpinupTimeoutSeconds + shootDurationSeconds + 0.5);

            return durationElapsed || safetyTimeout;
        }

        // In teleop mode, the command is held by trigger/whileTrue and cancelled upon release
        return false;
    }
}
