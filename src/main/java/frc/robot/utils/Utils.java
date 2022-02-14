package frc.robot.utils;

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

    /**
     * This function is a series of parabolas designed to smooth the input of the joystick.
     * Warning! This might not work well because the derivative of the output graph isn't continuous.
     *
     * @param joystickAcceleration is the change in the output of the joystick. [% / s]
     * @param joystickValue        is the value on which the smoothing function is applied. [%]
     * @return the smoothed value.
     */
    public static double smoothingFunction(double joystickAcceleration, double joystickValue) {
        final double kParabolaConstant = 0.75;
        double a, b = -1, c;

        if (joystickAcceleration < 0) {
            a = kParabolaConstant;
            c = (joystickValue < 0) ? 0 : 1;
        } else {
            a = -kParabolaConstant;
            c = (joystickValue < 0) ? -1 : 0;
        }

        return a * Math.pow(joystickValue, 2) + b * joystickValue + c;
    }
}