package frc.robot;


public final class Constants {

    public static final double NOMINAL_VOLTAGE = 12; // [V]

    public static class ExampleSubsystem {
        public static final double POWER = 0.5; // [%]
    }

    public static class Intake {
        public static final double DEFAULT_POWER = 0.5; // power intake will receive on the basic command. [%]
        public static final double POWER_TO_VELOCITY_RATIO = -3 / 16.0; // Ratio of power to velocity. [% / m/s]
        public static final boolean IS_COMPENSATING_VOLTAGE = true;
    }
}
