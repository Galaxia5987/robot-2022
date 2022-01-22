package frc.robot;

public final class Ports {
    public static class ExampleSubsystem {
        public static final int MAIN = 0;
        public static final int AUX = 0;
        public static final boolean IS_MAIN_INVERTED = false;
        public static final boolean IS_AUX_INVERTED = false;
        public static final boolean IS_MAIN_SENSOR_INVERTED = false;
        public static final boolean IS_AUX_SENSOR_INVERTED = false;
    }

    public static class Controls {
        public static final int JOYSTICK = 0;
        public static final int XBOX = 0;
    }

    /*
     left motor's data, right motor's data.
     */
    public static class Climber {
        public static final int LEFT = 0;
        public static final int RIGHT = 0;

        public static final boolean LEFT_SENSOR_PHASE = true;
        public static final boolean IS_LEFT_INVERTED = true;

        public static final boolean RIGHT_SENSOR_PHASE = true;
        public static final boolean IS_RIGHT_INVERTED = true;

        public static final int JOYSTICK_PORT = 0;

        /*
         Used only for simulation.
         */
        public static final int ENCODER_A_CHANNEL = 0;
        public static final int ENCODER_B_CHANNEL = 1;

    }
}