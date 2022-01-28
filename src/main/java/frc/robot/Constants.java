package frc.robot;


public final class Constants {

    public static final double NOMINAL_VOLTAGE = 12; // [V]

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Intake {
        public static final double POWER = 0.5; // power intake will receive on the basic command. [%]
        public static final double SLOPE = -3./16; // 0.5 is a temp number (still hasnt been tested). [% / m/s]
        public static final double BIAS = 1; // Maximal power of the intake. [%]
        public static final boolean IS_COMPENSATING_VOLTAGE = true;
    }
}
