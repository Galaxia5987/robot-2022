package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;

import java.util.function.DoubleUnaryOperator;
import java.util.function.Function;

public class Utils {


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

    public static double smoothed(double value, double exponent, LinearFilter joystickFilter) {
        double filteredValue = joystickFilter.calculate(value);
        return Math.pow(Math.abs(filteredValue), exponent) * Math.signum(filteredValue);
    }

    public static double thetaSmoothing(double val) {
        DoubleUnaryOperator function = x -> Math.pow(x + 1, 2) - 1;
        if (val > 0) {
            return -0.9 * function.applyAsDouble(-val) + 0.9 * function.applyAsDouble(Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        } else {
            return 0.9 * function.applyAsDouble(val) - 0.9 * function.applyAsDouble(Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        }
    }
}