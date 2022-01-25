package frc.robot;

public final class Constants {

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Climber {
        public static final double P_AUX_VELOCITY = 0;
        public static final double I_AUX_VELOCITY = 0;
        public static final double D_AUX_VELOCITY = 0;

        public static final double P_MAIN_VELOCITY = 0;
        public static final double I_MAIN_VELOCITY = 0;
        public static final double D_MAIN_VELOCITY = 0;

        public static final double P_AUX_POSITION = 0;
        public static final double I_AUX_POSITION = 0;
        public static final double D_AUX_POSITION = 0;

        public static final double P_MAIN_POSITION = 0;
        public static final double I_MAIN_POSITION = 0;
        public static final double D_MAIN_POSITION = 0;


        public static final double MAX_VELOCITY = Math.PI * 2 / 3; //[rad/s]

        public static final int GEAR_RATIO = 273;

        public static final double TICKS_PER_RAD = 2048 * GEAR_RATIO / (2 * Math.PI);

        public static final double THRESHOLD = 0.05; //[%]
    }
}
