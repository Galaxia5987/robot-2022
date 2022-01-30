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
     * Converts minutes into seconds.
     *
     * @param minutes is the input of minutes. [min]
     * @return how many seconds are in that minutes. [s]
     */
    public static double minutesToSeconds(double minutes) {
        return minutes * 60.0;
    }

    /**
     * Converts seconds into minutes.
     * Used for converting rpm into rps in shooter branch.
     *
     * @param seconds is the input of seconds. [s]
     * @return how many minutes the seconds amount to. [min]
     */
    public static double secondsToMinutes(double seconds) {
        return seconds / 60.0;
    }

}