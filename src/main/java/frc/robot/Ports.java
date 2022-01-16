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
        public static final int XBOX = 0;
    }

    public static class Intake {
        public static int MOTOR_PORT = 0;
        public static boolean MOTOR_INVERTED = false;
        public static int SOLENOID_PORT = 0;
    }
}
