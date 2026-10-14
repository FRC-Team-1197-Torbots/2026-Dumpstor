package frc.robot.Constants;

/**
 * Constants for the Intake Subsystem:
 * Includes CAN IDs, roller operational voltages, deploy position setpoints, and agitation parameters.
 */
public class IntakeConstants {
    // CANivore bus name for the intake mechanism
    public final static String CANLOOP = "can_s4";

    // CAN IDs for the Rack & Pinion Deploy motors (TalonFX)
    public final static int Intake1 = 35; // Master deploy motor
    public final static int Intake2 = 36; // Paired / inverted deploy motor

    // CAN IDs for the Intake Roller motors (TalonFX)
    public final static int Roller1 = 37; // Master roller motor
    public final static int Roller2 = 38; // Follower roller motor (opposed)

    // Roller Voltages (Volts):
    // Negative voltage drives rollers inward (intaking); positive drives outward (ejecting).
    // FRC "Touch it, own it" principle: surface speed should be 2x-3x drivetrain speed.
    public final static double kIntakeRollerVoltage = -9.0; 
    public final static double kEjectRollerVoltage = 6.0;
    public final static double kSlowIntakeVoltage = -4.0;
    public final static double kPulseReverseVoltage = 10.0;

    // Slot 0 Proportional Gain (kP) for deploy position closed loop
    public final static double KP = 1.2f; 

    // Deploy Position Setpoints (Rotations):
    // 0.0 rotations = fully stowed inside robot perimeter
    // 12.75 rotations = fully deployed ground intake extension
    public final static double kDeployTargetRots = 12.75;

    // Agitation Parameters:
    // Halfway deploy position (~50% extension) to shake and unbridge jammed game pieces
    public final static double kAgitateTargetRots = 6.375;
    // Time in seconds for each half of the agitation oscillation cycle (in / out)
    public final static double kAgitateCycleTime = 0.30;
    // Default duration in seconds for standalone autonomous agitation
    public final static double kDefaultAgitateDurationSeconds = 1.5;
}
