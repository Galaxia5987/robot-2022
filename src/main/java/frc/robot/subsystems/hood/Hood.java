package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.Hood.SOLENOID;

public class Hood extends SubsystemBase {
    private static Hood INSTANCE;
    private final Solenoid angleChanger = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);

    public Hood() {
    }

    /**
     * Gets the instance of the hood subsystem.
     *
     * @return the hood subsystem.
     */
    public static Hood getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Hood();
        }
        return INSTANCE;
    }

    /**
     * Opens the solenoid.
     * This function simply toggles the mode of the
     * solenoid only if it's currently closed.
     */
    public void open() {
        if (angleChanger.get())
            angleChanger.toggle();

    }

    /**
     * Closes the solenoid.
     * This function simply toggles the mode of the
     * solenoid only if it's currently open.
     */
    public void close() {
        if (!angleChanger.get()) {
            angleChanger.toggle();
        }
    }

    /**
     * Returns whether the solenoid is active.
     *
     * @return a boolean representing the mode of the solenoid.
     */
    public boolean getActive() {
        return angleChanger.get();
    }
}
