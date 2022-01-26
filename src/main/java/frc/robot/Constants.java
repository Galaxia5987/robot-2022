package frc.robot;

public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // loop period. [s]
    public static final double g = 9.80665; // Gravity acceleration constant. [m/s^2]
    public static final double UPPER_TARGET_HEIGHT = 2.64; // Height of upper target. [m]
    public static final double PEBZNER_HEIGHT = 4.8; // Height of pebzner auditorium. [m]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations. [ms]
    public static final double NOMINAL_VOLTAGE = 12.0; // Nominal voltage. [V]


    public static class Shooter {
        public static final int TICKS_PER_REVOLUTION = 2048; // Ticks per revolution of the shooter motor. [tick]
        public static final double WHEEL_RADIUS = 0.1016; // Radius of the wheel. [m]
        public static final double Ka = 1; // Acceleration state space coefficient (placeholder).
        public static final double Kv = 1; // Velocity state space coefficient (placeholder).
        public static final double MODEL_TOLERANCE = 1e-6; // Model tolerance for state space.
        public static final double SENSOR_TOLERANCE = 1e-6; // Sensor tolerance for state space.

        public static final double J = 0.00218; // Moment of inertia for state space. This will be changed if the system will be in rps. [kg*m^2]
        public static final double GEAR_RATIO = 1; // Gear ratio for encoder.
        public static final double NEUTRAL_DEADBAND = 0.1; // [%]

        public static class Bits{
            public static final double OUTPUT_MULTIPLIER = 0.1;
        }
    }

    public static class Control {
        public static final double RIGHT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
    }

    public static class Leds{
        public static final int SWERVE_LED_LENGTH = 60;
    }
}
