package frc.robot;

public final class Constants {

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Climber {
        public static final double P_LEFT_VELOCITY= 0;
        public static final double I_LEFT_VELOCITY= 0;
        public static final double D_LEFT_VELOCITY= 0;

        public static final double P_RIGHT_VELOCITY = 0;
        public static final double I_RIGHT_VELOCITY = 0;
        public static final double D_RIGHT_VELOCITY = 0;

        public static final double P_LEFT_POSITION= 0;
        public static final double I_LEFT_POSITION= 0;
        public static final double D_LEFT_POSITION= 0;

        public static final double P_RIGHT_POSITION = 0;
        public static final double I_RIGHT_POSITION = 0;
        public static final double D_RIGHT_POSITION = 0;



        public static final int MAX_VELOCITY = 4; //[m/s]

        public static final int GEAR_RATIO = 620;

        public static final double TICKS_PER_RAD = 2048 * GEAR_RATIO / (2 * Math.PI);;

        public static final double THRESHOLD = 0.05; //[cm]
    }
}
