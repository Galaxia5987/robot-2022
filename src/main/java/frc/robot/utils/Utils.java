package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;

import java.util.function.DoubleUnaryOperator;

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

    /**
     * Method used for smoothing with any threshold.
     * See functions implemented here visually at {@link <a href="https://www.math3d.org/EQBPTCQu3">...</a>}
     * The y-axis represents the threshold.
     * Note that the function h(x,y) doesn't extend beyond a y-value of 0.5.
     * This is because I doubt you need a threshold larger than 0.5.
     *
     * @param val       the input value to smooth. [%]
     * @param threshold the threshold to apply. [%]
     * @return the smoothed value with the threshold.
     */
    public static double swerveSmoothing(double val, double threshold) {
        if (Math.abs(val) < threshold) {
            return 0;
        }

        final int X = 0;
        final int Y = 1;

        DoubleUnaryOperator Fx = x -> Math.pow(x + 1, 2) - 1;
        MultivariableFunction Gxy = (inputs) ->
                (-Fx.applyAsDouble(-inputs[X]) + Fx.applyAsDouble(-inputs[Y])) /
                        (1 + Fx.applyAsDouble(-inputs[Y]));
        MultivariableFunction Hxy = (inputs) ->
                Gxy.apply(Math.abs(inputs[X]), inputs[Y]) *
                        Math.signum(inputs[X]);

        return Hxy.apply(val, threshold);
    }
}