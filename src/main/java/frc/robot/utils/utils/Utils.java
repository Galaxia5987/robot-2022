package frc.robot.utils.utils;

public class Utils {
    /**
     * Gets the minimal error between two desired angles.
     *
     * @param targetAngle  the desired angle. [rad]
     * @param currentAngle current angle. [rad]
     * @return the distance between the angles. [rad]
     */
    public static double getTargetError(double targetAngle, double currentAngle) {
        double cwDistance = targetAngle - currentAngle;
        double ccwDistance = 2 * Math.PI - (Math.abs(cwDistance));
        if (Math.abs(cwDistance) < ccwDistance) {
            return cwDistance;
        } else if (cwDistance < 0) {
            return ccwDistance;
        }
        return -ccwDistance;
    }

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
     * sets the value of the joystick to 0 if the value is less than the threshold,
     *
     * @param input     the joystick value.
     * @param threshold the threshold value.
     * @return 0 if val is less than the threshold else the value between 0 and 1 or -1.
     */
    public static double rotationalDeadband(double input, double threshold) {
        if (Math.abs(input) < threshold)
            return 0;
        return (input - (Math.signum(input) * threshold)) / (1 - threshold);
    }

    /**
     * sets the joystick vector value to 1 if the value is greater than the threshold
     *
     * @param vec       the joystick value vector
     * @param threshold the threshold value
     * @return 1 if vec is greater than the threshold otherwise returns vec
     */
    public static double outerDeadzone(double vec, double threshold) {
        if (Math.abs(vec) > threshold)
            return 1;
        return vec;
    }

    /**
     * sets the angle deadzones of the joystick to the nearest 90 degree multiple if the error is less than the threshold
     *
     * @param alphaDeg  the joystick angle
     * @param threshold the threshold value
     * @return a multiple of 90 if the angle is in the deadzone otherwise return the angle
     */
    public static double angleDeadZones(double alphaDeg, double threshold) {
        double div = alphaDeg / 90;
        double errorDeg = (Math.round(div) - div) * 90;
        if (errorDeg < threshold)
            return alphaDeg + errorDeg;
        return alphaDeg;
    }

    /**
     * Clamp the value between the [min, max] range.
     *
     * @param value the value to check.
     * @param min   the minimal value.
     * @param max   the maximal value.
     * @return the value clamped.
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
