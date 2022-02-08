package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

/**
 * This class contains a boolean value and a timer. It can set its boolean value and return whether the timer is within
 * a set timeout. These returns true if the stored value is true and the timeout has expired.
 * <p>
 * took from team 254 code.
 */
public class TimeDelayedBoolean {
    private final Timer timer = new Timer();
    private boolean old = false;

    public boolean update(boolean value, double timeout) {
        if (!old && value) { // the state changed -> reset the timer.
            timer.reset();
            timer.start();
        }
        old = value;
        return value && timer.hasElapsed(timeout);
    }
}