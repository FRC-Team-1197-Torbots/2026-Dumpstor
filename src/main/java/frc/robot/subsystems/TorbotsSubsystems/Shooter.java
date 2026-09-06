package frc.robot.subsystems.TorbotsSubsystems;

import org.wpilib.command2.SubsystemBase;

public class Shooter extends SubsystemBase {
    private Drum m_drum;
    private Kicker m_kicker;

    public Shooter() {
        m_drum = new Drum();
        m_kicker = new Kicker();
    }
}
