package frc.robot.utils;

public class Units {
    /**
     * Converts the value from m/s to rps.
     *
     * @param value  the speed in m/s.
     * @param radius the radius of the wheel. [m]
     * @return the speed in rps.
     */
    public static double metersPerSecondToRps(double value, double radius) {
        return value / (2 * Math.PI * radius);
    }
}
