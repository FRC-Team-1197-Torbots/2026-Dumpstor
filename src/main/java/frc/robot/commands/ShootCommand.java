package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TorbotsSubsystems.Drum;
import frc.robot.subsystems.TorbotsSubsystems.Floor;
import frc.robot.subsystems.TorbotsSubsystems.Intake;
import frc.robot.subsystems.TorbotsSubsystems.Kicker;

public class ShootCommand extends Command {
    private final Drum drum;
    private final Floor floor;
    private final Kicker kicker;
    private final Intake intake;
    private final double targetRPS;
    private final double shootDurationSeconds;
    private final boolean agitateIntake;

    // Tolerance in RPS.
    private static final double RPS_TOLERANCE = 2.0;

    private boolean isShooting = false;
    private final Timer spinupTimer = new Timer();
    private final Timer shootDurationTimer = new Timer();

    /**
     * Teleop constructor: continuous shooting (finishes when trigger is released).
     * Intake agitation is false so operator can manually agitate if desired.
     */
    public ShootCommand(Drum drum, Floor floor, Kicker kicker, Intake intake, double targetRPS) {
        this(drum, floor, kicker, intake, targetRPS, -1.0, false);
    }

    /**
     * Autonomous constructor: feeds for a set duration after spin-up, then finishes cleanly.
     * Optionally agitates intake while shooting.
     *
     * @param shootDurationSeconds Number of seconds to feed kicker/rollers after reaching speed.
     *                             Set <= 0 for continuous shooting.
     * @param agitateIntake        Whether to oscillate the intake deploy while shooting.
     */
    public ShootCommand(
        Drum drum,
        Floor floor,
        Kicker kicker,
        Intake intake,
        double targetRPS,
        double shootDurationSeconds,
        boolean agitateIntake
    ) {
        this.drum = drum;
        this.floor = floor;
        this.kicker = kicker;
        this.intake = intake;
        this.targetRPS = targetRPS;
        this.shootDurationSeconds = shootDurationSeconds;
        this.agitateIntake = agitateIntake;

        // Ensure this command claims the subsystems so nothing else tries to use them
        addRequirements(drum, floor, kicker, intake);
    }

    @Override
    public void initialize() {
        isShooting = false;
        spinupTimer.restart();
        shootDurationTimer.reset();

        drum.setVelocityRPS(targetRPS);
        floor.stop();
        kicker.stop();
        intake.stopRollers();
        intake.setDeployPosition(IntakeConstants.kDeployTargetRots);
    }

    @Override
    public void execute() {
        org.wpilib.telemetry.Telemetry.log("Auto/isShooting", isShooting);
        org.wpilib.telemetry.Telemetry.log("Auto/TargetRPS", targetRPS);
        org.wpilib.telemetry.Telemetry.log("Shooter/VelocityRPS", drum.getVelocityRPS());

        // Latch isShooting to true when drum reaches target speed OR fallback spinup timeout passes
        if (!isShooting) {
            boolean atSpeed = Math.abs(drum.getVelocityRPS() - targetRPS) <= RPS_TOLERANCE;
            boolean spinupTimeout = spinupTimer.hasElapsed(ShooterConstants.kSpinupTimeoutSeconds);

            if (atSpeed || spinupTimeout) {
                isShooting = true;
                shootDurationTimer.restart();
            }
        }

        if (isShooting) {
            kicker.setVoltage(ShooterConstants.kKickerFeedVoltage);
            intake.setRollerVoltage(IntakeConstants.kIntakeRollerVoltage);

            // Automatically agitate the intake deploy back and forth while feeding
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

    @Override
    public void end(boolean interrupted) {
        // Return drum to idle speed and turn feed mechanisms off when finished or canceled
        drum.setVelocityRPS(ShooterConstants.IdleShooterSpeed);
        floor.stop();
        kicker.stop();
        intake.stopRollers();
        intake.setDeployPosition(IntakeConstants.kDeployTargetRots);
    }

    @Override
    public boolean isFinished() {
        // In autonomous mode: finish once active shooting duration has elapsed,
        // or safety fallback timeout elapses so autonomous never blocks
        if (shootDurationSeconds > 0) {
            boolean durationElapsed = isShooting && shootDurationTimer.hasElapsed(shootDurationSeconds);
            boolean safetyTimeout = spinupTimer.hasElapsed(ShooterConstants.kSpinupTimeoutSeconds + shootDurationSeconds + 0.5);
            return durationElapsed || safetyTimeout;
        }

        // In teleop mode: held by button and canceled when released
        return false;
    }
}
