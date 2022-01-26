package frc.robot;


public final class Constants {
    public static final double NOMINAL_VOLTAGE = 12; // [volts]

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Climber {
        public static final double KP = 1.5;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double F_FORWARD_S = 0;
        public static final double F_FORWARD_COS = 0;
        public static final double F_FORWARD_V = 0;
        public static final double F_FORWARD_A = 0;

        public static final double CRUISE_VELOCITY = 0; // [ticks/100ms]
        public static final double MAXIMAL_ACCELERATION = 0; // [ticks/100ms*sec]
        public static final boolean VOLTAGE_COMPENSATION = true;

        public static final double MAX_VELOCITY = Math.PI * 2 / 3; //[rad/s]

        public static final double GEAR_RATIO = 292.1;

        public static final double TICKS_PER_RAD = 2048 * GEAR_RATIO / (2 * Math.PI);

        public static final double JOYSTICK_DEADBAND = 0.05; // [%]

        public static final double ARM_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 2048;
        public static final double ARM_MASS = 5.0; // Kilograms
        public static final double ARM_LENGTH = 0.792; // [m]

        public static final double MAX_ANGLE = Math.toRadians(255); // [radians]
        public static final double MIN_ANGLE = Math.toRadians(-75); // [radians]

        public static final int AUTONOMOUS_TIME = 15; // [s]

        public static final double STOP_CLIMBER = 149.5; // [s]
    }
}
