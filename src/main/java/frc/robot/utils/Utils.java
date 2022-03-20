package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;

import java.util.function.DoubleUnaryOperator;

public class Utils {


    /**
     * This method ensures that any accidental input is set to 0.
     * In other words, if the value is less than the threshold the function returns 0.
     *
     * @param val       the input value.
     * @param threshold the threshold value.
     * @return 0 if the value is less than the threshold else the value.
     */
public static double conventionalDeadband(double val, double threshold) {
        if (Math.abs(val) < threshold)
            return 0;
        return val;
    }

    /**
     * @param val       the input value.
     * @param threshold the threshold value.
     * @return the updated value after the deadband.
     */
    public static double continuousDeadband(double val, double threshold) {
        val = conventionalDeadband(val, threshold);
        return (val - (Math.signum(val) * threshold)) / (1 - threshold);
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
     * Smoothing method making use of a moving average filter,
     * fulfilling the purpose of slowing down an input and filtering noise.
     *
     * @param value               the value to smooth.
     * @param exponent            the exponent to smooth the value by.
     * @param movingAverageFilter the filter to lower input noise.
     * @return the smoothed value.
     */
    public static double smoothed(double value, double exponent, LinearFilter movingAverageFilter) {
        double filteredValue = movingAverageFilter.calculate(value);
        return Math.pow(Math.abs(filteredValue), exponent) * Math.signum(filteredValue);
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

        /*
        All mathematical functions implemented here are an exact replica
        of the functions displayed visually in the link above.
         */
        DoubleUnaryOperator Fx = x -> Math.pow(x + 1, 2) - 1;
        MultivariableFunction Gxy = (inputs) ->
                (-Fx.applyAsDouble(-inputs[X]) + Fx.applyAsDouble(-inputs[Y])) /
                        (1 + Fx.applyAsDouble(-inputs[Y]));
        MultivariableFunction Hxy = (inputs) ->
                Gxy.apply(Math.abs(inputs[X]), inputs[Y]) *
                        Math.signum(inputs[X]);

        return Hxy.apply(val, threshold);
    }

    public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}