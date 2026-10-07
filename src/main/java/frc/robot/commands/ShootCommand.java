package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
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

    // Tolerance in RPS. Adjust this to be tighter or looser depending on how
    // quickly
    // the flywheels recover and how precise you need the shot.
    private static final double RPS_TOLERANCE = 2.0;

    public ShootCommand(Drum drum, Floor floor, Kicker kicker, Intake intake, double targetRPS) {
        this.drum = drum;
        this.floor = floor;
        this.kicker = kicker;
        this.intake = intake;
        this.targetRPS = targetRPS;

        // Ensure this command claims the subsystems so nothing else tries to use them
        addRequirements(drum, floor, kicker, intake);
    }

    private boolean isShooting = false;
    private final Timer agitationTimer = new Timer();
    private boolean isAgitating = false;
    private boolean hasAgitatedOnce = false;
    private final Timer timeoutTimer = new Timer();

    @Override
    public void initialize() {
        isShooting = false;
        isAgitating = false;
        hasAgitatedOnce = false;
        agitationTimer.stop();
        agitationTimer.reset();
        timeoutTimer.restart();
        drum.setVelocityRPS(targetRPS);
        floor.stop();
        kicker.stop();
        intake.stopRollers();
    }

    @Override
    public void execute() {
        drum.setVelocityRPS(targetRPS);

        // If we reach speed or 1 second passes, latch the isShooting boolean to true forever (until
        // command ends)
        if (!isShooting && (Math.abs(drum.getVelocityRPS() - targetRPS) <= RPS_TOLERANCE || timeoutTimer.hasElapsed(1.0))) {
            isShooting = true;
            agitationTimer.restart();
        }

        if (isShooting) {
            // floor.setVoltage(ShooterConstants.kFloorFeedVoltage);
            kicker.setVoltage(ShooterConstants.kKickerFeedVoltage);
            intake.setRollerVoltage(frc.robot.Constants.IntakeConstants.kIntakeRollerVoltage);
            
            double waitTime = hasAgitatedOnce ? 0.75 : 1.5;
            
            // Agitation logic: wait initially 2.5s, then every 0.75s
            if (!isAgitating && agitationTimer.hasElapsed(waitTime)) {
                isAgitating = true;
                agitationTimer.restart();
            }

            if (isAgitating) {
                double targetHalfway = frc.robot.Constants.IntakeConstants.kDeployTargetRots / 2.0;
                intake.setDeployPosition(targetHalfway);
                
                // If it has reached the halfway point (with a 0.5 rotation tolerance), extend back out
                if (intake.getDeployPosition() <= targetHalfway + 0.5) {
                    isAgitating = false;
                    hasAgitatedOnce = true;
                    agitationTimer.restart();
                }
            } else {
                intake.setDeployPosition(frc.robot.Constants.IntakeConstants.kDeployTargetRots);
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Turn everything off when the command finishes or is canceled
        floor.stop();
        kicker.stop();
        intake.stopRollers();
        intake.setDeployPosition(frc.robot.Constants.IntakeConstants.kDeployTargetRots);
    }

    @Override
    public boolean isFinished() {
        // Shoot commands are typically held by a button and cancel when released.
        // Return false so it runs continuously as long as the button is held.
        return false;
    }
}
