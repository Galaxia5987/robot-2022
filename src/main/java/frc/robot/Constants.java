package frc.robot;


public final class Constants {

    public static final double NOMINAL_VOLTAGE = 12;

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Intake {
        public static final double POWER = 0.5; // [%], power intake will receive on the basic command
        public static final double SLOPE = -3./16; // [% / m/s], 0.5 is a temp number (still hasnt been tested).
        public static final double BIAS = 1;

    }
}
