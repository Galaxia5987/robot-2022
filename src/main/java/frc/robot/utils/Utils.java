package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;

public class Utils {
    private static final LinearFilter joystickFilter = LinearFilter.movingAverage(
            Constants.Control.JOYSTICK_FILTER_TAP);

    /**
     * Sets the value of the joystick to 0 if the value is less than the threshold.
     *
     * @param val       the joystick value.
     * @param threshold the threshold value.
     * @return 0 if the value is less than the threshold else the value.
     */
    public static double deadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return val;
    }

    /**
     * @param input     the joystick input.
     * @param threshold the joystick deadband threshold.
     * @return the updated value after the deadband.
     */
    public static double rotationalDeadband(double input, double threshold) {
        if (Math.abs(input) < threshold)
            return 0;
        return (input - (Math.signum(input) * threshold)) / (1 - threshold);
    }

    /**
     * This function converts rps into rpm.
     *
     * @param rps is the input velocity. [rps]
     * @return the same velocity. [rpm]
     */
    public static double rpsToRpm(double rps) {
        return rps * 60.0;
    }


    /**
     * This function converts rps into rpm.
     *
     * @param rpm is the input velocity. [rpm]
     * @return the same velocity. [rps]
     */
    public static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }

    /**
     * This smoothing function includes a simple averaging linear filter,
     * and a regular smoothing mathematical function for an extra smooth experience.
     * 
     * @param val is the input parameter of the joystick. [%]
     * @return the premium smoothed value. [%]
     */
    public static double joystickSmoothing(double val) {
        double filteredValue = joystickFilter.calculate(val);
        return Math.pow(Math.abs(filteredValue), Constants.Control.JOYSTICK_SMOOTHING_EXPONENT) * Math.signum(filteredValue);
    }
}