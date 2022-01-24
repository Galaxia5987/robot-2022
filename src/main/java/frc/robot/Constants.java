package frc.robot;


public final class Constants {

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Climber {
        public static final double P_VELOCITY = 1.5;
        public static final double I_VELOCITY = 0;
        public static final double D_VELOCITY = 0;

        public static final double F_FORWARD_S = 0;
        public static final double F_FORWARD_COS = 0;
        public static final double F_FORWARD_V = 0;
        public static final double F_FORWARD_A = 0;

        public static final double MAX_ACCELERATION = 0;
        public static final double MIN_ACCELERATION = 0;

        public static final double MAX_VELOCITY = Math.PI * 2 / 3; //[rad/s]

        public static final double GEAR_RATIO = 292.1;

        public static final double TICKS_PER_RAD = 2048 * GEAR_RATIO / (2 * Math.PI);

        public static final double THRESHOLD = 0.05; //[%]

        public static final double ARM_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 2048;
        public static final double ARM_MASS = 5.0; // Kilograms
        public static final double ARM_LENGTH = 0.792; // [m]

        public static final int MAX_ANGLE = 255;
        public static final int MIN_ANGLE = -75;

        public static final int AUTONOMY_TIME = 15;

        public static final double STOP_CLIMBER = 249.5; // [s]
    }
}
