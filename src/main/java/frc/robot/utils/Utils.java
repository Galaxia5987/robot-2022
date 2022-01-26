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
     * This function converts a certain color into an array of rgb values.
     *
     * @param color is the array of rgb values.
     * @return return color array.
     */
    public static int[] getRGBValues(String color) {
        int[] rgb;
        switch (color) {
            case "Red":
                rgb = new int[]{255, 0, 0};
                break;
            case "Green":
                rgb = new int[]{0, 255, 0};
                break;
            case "Blue":
                rgb = new int[]{0, 0, 255};
                break;
            case "Purple":
                rgb = new int[]{126, 82, 160};
                break;
            case "Yellow":
                rgb = new int[]{255, 212, 0};
                break;
            case "Orange":
                rgb = new int[]{255, 159, 28};
                break;
            default:
                rgb = new int[]{0, 0, 0};
        }
        return rgb;
    }
}