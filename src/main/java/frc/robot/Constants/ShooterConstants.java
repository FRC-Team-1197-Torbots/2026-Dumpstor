package frc.robot.Constants;

//shooter contains, drum, kicker, floor
public class ShooterConstants {
    public final static int CANLOOP = 1;

    public final static int CANDrumRight1 = 1;
    public final static int CANDrumRight2 = 2;
    public final static int CANDrumLeft1 = 3;
    public final static int CANDrumLeft2 = 4;

    public final static float KS = 0.0f;
    public final static float KV = 0.0f;
    public final static float KD = 0.0f;

    public final static int CANRange1 = 30;
    public final static int CANRange2 = 31;

    public final static int CANKick1 = 5;
    public final static int CANKick2 = 6;

    public final static int CANFloor1 = 7;
    public final static int CANFloor2 = 8;

    // Floor/Hopper speeds
    // Feed speed should be fast enough to prevent gaps between game pieces 
    // but not so fast it overwhelms the kicker/shooter.
    public final static double kFloorFeedVoltage = 12.0;
    public final static double kFloorUnjamVoltage = -6.0;

    // Kicker speeds
    // The kicker accelerates the game piece into the main flywheels. 
    // It should be fast to maintain shot velocity, usually 8V-12V.
    public final static double kKickerFeedVoltage = -7.0;
    public final static double kKickerUnjamVoltage = 6.0;

    public final static double IdleShooterSpeed = 4000/60;
}
