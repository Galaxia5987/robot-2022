package frc.robot;

public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // loop period. [s]
    public static final double g = 9.80665; // Gravity acceleration constant. [m/s^2]
    public static final double UPPER_TARGET_HEIGHT = 2.64; // Height of upper target. [m]
    public static final double PEBZNER_HEIGHT = 4.8; // Height of pebzner auditorium. [m]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations. [ms]
    public static final double NOMINAL_VOLTAGE = 12.0; // Nominal voltage. [V]


    public static class Shooter {
        public static final double TICKS_PER_REVOLUTION = 2048; // Ticks per revolution of the shooter motor. [tick]
        public static final double WHEEL_RADIUS = 0.1016; // Radius of the wheels. [m]

        public static final double Ka = 1; // Acceleration state space coefficient (placeholder).
        public static final double Kv = 1; // Velocity state space coefficient (placeholder).
        public static final double MODEL_TOLERANCE = 10; // Model tolerance for state space.
        public static final double SENSOR_TOLERANCE = 0.1; // Sensor tolerance for state space.
        public static final double VELOCITY_TOLERANCE = 0.15; // Velocity tolerance for state space.
        public static final double COST_LQR = 55; // Cost lqr for state space.

        public static final double J = 0.00218; // Moment of inertia for state space. [kg*m^2]
        public static final double GEAR_RATIO = 1; // Gear ratio for encoder (placeholder).
        public static final double NEUTRAL_DEADBAND = 0.1; // [%]

        public static final double OUTPUT_MULTIPLIER = 0.1; // Multiplies the output for manual control in the bits. [%]
    }

    public static class Control {
        public static final double RIGHT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
    }
}
