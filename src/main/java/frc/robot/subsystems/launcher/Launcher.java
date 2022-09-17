package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    private static Launcher INSTANCE;

    public static Launcher getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Launcher();
        }
        return INSTANCE;
    }
}
