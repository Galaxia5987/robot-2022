package frc.robot.subsystems.flap;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.Conveyor.SOLENOID;

public class Flap extends SubsystemBase {
    private final Solenoid flap = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);
    private static Flap INSTANCE;

    public Flap() {
    }

    public static Flap getInstance() {
        if(INSTANCE == null){
            INSTANCE = new Flap();
        }
        return INSTANCE;
    }

    /**
     * open the flap
     */
    public void openFlap() {
        flap.set(false);
    }

    /**
     * closes the flap
     */
    public void closeFlap() {
        flap.set(true);
    }

    /**
     * toggles the flap
     */
    public void toggleFlap() {
        flap.toggle();
    }

    /**
     * Gets the current mode of the flap.
     *
     * @return the mode of the flap (true or false).
     */
    public boolean getFlapMode() {
        return flap.get();
    }

    /**
     * Sets the mode of the flap (open or closed).
     *
     * @param flapMode is the enum describing the mode of the flap.
     */
    public void setFlapMode(FlapMode flapMode) {
        flap.set(flapMode.mode);
    }

    public enum FlapMode {
        Open(false),
        Closed(true);

        public final boolean mode;

        FlapMode(boolean mode) {
            this.mode = mode;
        }

        /**
         * Get a flap mode according to input boolean (true - closed, false - open).
         *
         * @param val is the input value.
         * @return the flap mode as an enum.
         */
        public static FlapMode getValue(boolean val) {
            if (val) {
                return Closed;
            }
            return Open;
        }
    }
}
