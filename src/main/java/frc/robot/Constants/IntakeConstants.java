package frc.robot.Constants;

//Rack and roller
public class IntakeConstants {
    public final static String CANLOOP = "can_s4";

    public final static int Intake1 = 35;
    public final static int Intake2 = 36;

    public final static int Roller1 = 37;
    public final static int Roller2 = 38;

    // FRC "Touch it, own it" principle: surface speed should be 2x-3x drivetrain speed.
    // Assuming a 2-inch roller on a Falcon/Kraken, 8V to 10V typically achieves this.
    public final static double kIntakeRollerVoltage = -9.0; 
    public final static double kEjectRollerVoltage = 6.0;

    public final static double KP = 1.2f; 

    // Deploy rotations
    public final static double kDeployTargetRots = 12.75;
}
