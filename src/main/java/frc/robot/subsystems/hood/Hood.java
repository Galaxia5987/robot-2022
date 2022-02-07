package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.Hood.SOLENOID;

public class Hood extends SubsystemBase {
    private static Hood INSTANCE;
    private final Solenoid angleChanger = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);

    private Hood() {
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
        angleChanger.set(false);
    }

    /**
     * Closes the solenoid.
     * This function simply toggles the mode of the
     * solenoid only if it's currently open.
     */
    public void close() {
        angleChanger.set(true);
    }

    public void setSolenoid(Mode mode) {
        angleChanger.set(mode.value);
    }

    /**
     * Returns whether the solenoid is open.
     *
     * @return whether the solenoid is open.
     */
    public boolean isOpen() {
        return angleChanger.get();
    }

    public enum Mode {
        ShortDistance(true),
        LongDistance(false);

        public final boolean value;

        Mode(boolean value) {
            this.value = value;
        }

        public static Mode getValue(boolean val) {
            if (val) {
                return ShortDistance;
            } else {
                return LongDistance;
            }
        }
    }
}
