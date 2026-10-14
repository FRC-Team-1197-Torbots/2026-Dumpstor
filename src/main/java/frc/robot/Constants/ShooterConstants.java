package frc.robot.Constants;

/**
 * Constants for the Shooter Mechanism:
 * Includes CAN IDs and operational setpoints for Drum (Flywheels), Kicker, and Floor.
 */
public class ShooterConstants {
    // CANivore bus name for the shooter mechanism
    public final static String CANLOOP = "can_s1";

    // CAN IDs for the 4 Drum Flywheel motors (Kraken / Falcon)
    public final static int CANDrumRight1 = 1; // Master motor
    public final static int CANDrumRight2 = 2; // Follower motor (aligned)
    public final static int CANDrumLeft1 = 3;  // Follower motor (opposed)
    public final static int CANDrumLeft2 = 4;  // Follower motor (opposed)

    // CAN IDs for the 2 Kicker roller motors
    public final static int CANKick1 = 5; // Master kicker motor
    public final static int CANKick2 = 6; // Follower kicker motor (opposed)

    // CAN IDs for the 2 Floor conveyor motors
    public final static int CANFloor1 = 7; // Master floor motor
    public final static int CANFloor2 = 8; // Follower floor motor (opposed)

    // Floor/Hopper control:
    // Set to true if the physical robot uses floor conveyor belts to advance balls into kicker.
    // Set to false if balls feed directly from intake rollers into kicker (current setup).
    public final static boolean kUseFloorFeed = false;
    public final static double kFloorFeedVoltage = 4.0;
    public final static double kFloorUnjamVoltage = -6.0;

    // Kicker speeds (Volts)
    // The kicker accelerates the game piece directly into the main flywheels.
    public final static double kKickerFeedVoltage = -9.0;
    public final static double kKickerUnjamVoltage = 9.0;

    // Shooter Speeds in Rotations Per Second (RPS = RPM / 60.0)
    public final static double IdleShooterSpeed = 3000.0 / 60.0; // 50 RPS (3000 RPM idle)
    public final static double LowShooterSpeed = 4000.0 / 60.0;  // 66.67 RPS (4000 RPM standard shot)
    public final static double MaxShooterSpeed = 4800.0 / 60.0;  // 80 RPS (4800 RPM high shot)

    // Autonomous shooting duration (seconds to actively feed kicker/rollers in auto)
    public final static double kAutoShootDurationSeconds = 2.5;

    // Fallback timeout to start feeding even if target RPS isn't strictly reached (prevents auto hangs)
    public final static double kSpinupTimeoutSeconds = 1.2;
}


