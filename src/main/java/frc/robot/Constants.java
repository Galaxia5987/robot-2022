package frc.robot;


public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // loop period. [s]
    public static final double g = 9.80665; // Gravity acceleration constant. [m/s^2]
    public static final double UPPER_TARGET_HEIGHT = 2.64; // Height of upper target. [m]
    public static final double PEBZNER_HEIGHT = 4.8; // Height of pebzner auditorium. [m]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].

    public static class Hood {
        public static final double ACTIVE_ANGLE = 55; // Angle of the shooter when the angle changer is active (placeholder). [deg]
        public static final double INACTIVE_ANGLE = 70; // Angle of the shooter when the angle changer is inactive (placeholder). [deg]
    }
}
