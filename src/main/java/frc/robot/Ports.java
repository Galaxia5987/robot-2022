package frc.robot;

public final class Ports {
    public static class ExampleSubsystem{
        public static final int MAIN = 0;
        public static final int AUX = 0;
        public static final boolean IS_MAIN_INVERTED = false;
        public static final boolean IS_AUX_INVERTED = false;
        public static final boolean IS_MAIN_SENSOR_INVERTED = false;
        public static final boolean IS_AUX_SENSOR_INVERTED = false;
    }

    public static class Shooter {
        public static final int MAIN_MOTOR = 0; // Main motor port.
        public static final boolean IS_MAIN_INVERTED = false; // Whether the motor is inverted.
        public static final boolean MAIN_SENSOR_PHASE = false; // Whether the encoder is inverted.
    }

    public static class Controls {
        public static final int XBOX = 0; // Xbox controller port.
    }

}
