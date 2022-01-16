package frc.robot.utils;

public class Utils {

    /**
     * sets the value of the joystick to 0 if the value is less than the threshold
     *
     * @param val       the joystick value
     * @param threshold the threshold value
     * @return 0 if val is less than the threshold else val
     */
    public static double deadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return val;
    }

    /**
     * @param input     the joystick input
     * @param threshold the joystick deadband threshold
     * @return the updated value after the deadband
     */
    public static double rotationalDeadband(double input, double threshold) {
        if (Math.abs(input) < threshold)
            return 0;
        return (input - (Math.signum(input) * threshold)) / (1 - threshold);
    }

    /**
     * This function makes sure that the input parameter val is in a certain range.
     *
     * @param val is the value to keep in said certain rage.
     * @param min is the minimal value of the input parameter val.
     * @param max is the maximal value of the input parameter val.
     * @return the fixed value.
     */
    public static double clamp(double val, double min, double max) {
        return Math.min(Math.max(val, min), max);
    }
}