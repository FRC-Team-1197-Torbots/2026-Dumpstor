package frc.robot.commands;

import org.wpilib.command2.Command;

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

    // Tolerance in RPS. Adjust this to be tighter or looser depending on how quickly 
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

    @Override
    public void initialize() {
        // Start spinning the drum up to speed immediately
        drum.setVelocityRPS(targetRPS);
        
        // Ensure feeders are stopped while we wait for spin-up
        floor.stop();
        kicker.stop();
        intake.stopRollers();
    }

    @Override
    public void execute() {
        // Keep commanding the drum to its target
        drum.setVelocityRPS(targetRPS);

        // Check if the drum is up to speed
        double currentRPS = drum.getVelocityRPS();
        boolean isAtSpeed = Math.abs(currentRPS - targetRPS) <= RPS_TOLERANCE;

        if (isAtSpeed) {
            // Drum is ready, feed the game piece!
            floor.setVoltage(ShooterConstants.kFloorFeedVoltage);
            kicker.setVoltage(ShooterConstants.kKickerFeedVoltage);
            intake.setRollerVoltage(frc.robot.Constants.IntakeConstants.kIntakeRollerVoltage);
        } else {
            // Drum is still spinning up or recovering from a shot, pause feeding
            floor.stop();
            kicker.stop();
            intake.stopRollers();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Turn everything off when the command finishes or is canceled
        floor.stop();
        kicker.stop();
        intake.stopRollers();
    }

    @Override
    public boolean isFinished() {
        // Shoot commands are typically held by a button and cancel when released.
        // Return false so it runs continuously as long as the button is held.
        return false;
    }
}
