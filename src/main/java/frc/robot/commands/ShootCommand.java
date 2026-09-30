package frc.robot.commands;

import org.wpilib.command2.Command;
import edu.wpi.first.wpilibj.Timer;

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

    @Override
    public void initialize() {
        isShooting = false;
        isAgitating = false;
        agitationTimer.stop();
        agitationTimer.reset();
        drum.setVelocityRPS(targetRPS);
        floor.stop();
        kicker.stop();
        intake.stopRollers();
    }

    @Override
    public void execute() {
        drum.setVelocityRPS(targetRPS);

        // If we reach speed, latch the isShooting boolean to true forever (until
        // command ends)
        if (!isShooting && Math.abs(drum.getVelocityRPS() - targetRPS) <= RPS_TOLERANCE) {
            isShooting = true;
            agitationTimer.restart();
        }

        if (isShooting) {
            floor.setVoltage(ShooterConstants.kFloorFeedVoltage);
            kicker.setVoltage(ShooterConstants.kKickerFeedVoltage);
            intake.setRollerVoltage(frc.robot.Constants.IntakeConstants.kIntakeRollerVoltage);
            
            // Agitation logic: every 2.5 seconds, pull halfway in for 0.5 seconds
            if (!isAgitating && agitationTimer.hasElapsed(2.5)) {
                isAgitating = true;
                agitationTimer.restart();
            }

            if (isAgitating) {
                double targetHalfway = frc.robot.Constants.IntakeConstants.kDeployTargetRots / 2.0;
                intake.setDeployPosition(targetHalfway);
                
                // If it has reached the halfway point (with a 0.5 rotation tolerance), extend back out
                if (intake.getDeployPosition() <= targetHalfway + 0.5) {
                    isAgitating = false;
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
