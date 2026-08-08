package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    private Drum drum;
    private Kicker kicker;

    public Shooter() {
        drum = new Drum();
        kicker = new Kicker();
    }

    public void KickOn() {
        kicker.On();
    }

    public void KickOff() {
        kicker.Off();
    }

    public void TestDrum() {
        drum.TestDrum();
    }

    public void StopDrum() {
        drum.DrumOff();
    }
}
