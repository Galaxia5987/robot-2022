package frc.robot.subsystems.flap;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Ports.Flap.SOLENOID;

public class Flap extends SubsystemBase {
    private static Flap INSTANCE;
    private final Solenoid flap = new Solenoid(PneumaticsModuleType.CTREPCM, SOLENOID);

    private Flap() {
    }

    public static Flap getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Flap();
        }
        return INSTANCE;
    }

    /**
     * open the flap
     */
    public void allowShooting() {
        flap.set(FlapMode.ALLOW_SHOOTING.mode);
    }

    /**
     * closes the flap
     */
    public void blockShooter() {
        flap.set(FlapMode.STOP_CARGO.mode);
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
    public FlapMode getFlapMode() {
        return FlapMode.getValue(flap.get());
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
        ALLOW_SHOOTING(true),
        STOP_CARGO(false);

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
            if (!val) {
                return STOP_CARGO;
            }
            return ALLOW_SHOOTING;
        }
    }
}
