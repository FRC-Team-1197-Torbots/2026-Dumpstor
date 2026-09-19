package frc.robot.commands;

import org.wpilib.command2.Command;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TorbotsSubsystems.Drum;
import frc.robot.subsystems.TorbotsSubsystems.Floor;
import frc.robot.subsystems.TorbotsSubsystems.Kicker;

public class ShootCommand extends Command {
    private final Drum drum;
    private final Floor floor;
    private final Kicker kicker;
    private final double targetRPS;

    // Tolerance in RPS. Adjust this to be tighter or looser depending on how quickly 
    // the flywheels recover and how precise you need the shot.
    private static final double RPS_TOLERANCE = 2.0; 

    public ShootCommand(Drum drum, Floor floor, Kicker kicker, double targetRPS) {
        this.drum = drum;
        this.floor = floor;
        this.kicker = kicker;
        this.targetRPS = targetRPS;

        // Ensure this command claims the subsystems so nothing else tries to use them
        addRequirements(drum, floor, kicker);
    }

    @Override
    public void initialize() {
        // Start spinning the drum up to speed immediately
        drum.setVelocityRPS(targetRPS);
        
        // Ensure feeders are stopped while we wait for spin-up
        floor.stop();
        kicker.stop();
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
        } else {
            // Drum is still spinning up or recovering from a shot, pause feeding
            floor.stop();
            kicker.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Turn everything off when the command finishes or is canceled
        drum.stop();
        floor.stop();
        kicker.stop();
    }

    @Override
    public boolean isFinished() {
        // Shoot commands are typically held by a button and cancel when released.
        // Return false so it runs continuously as long as the button is held.
        return false;
    }
}
