package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public final class Ports {
    public static class ExampleSubsystem {
        public static final int MAIN = 0;
        public static final int AUX = 0;
        public static final boolean IS_MAIN_INVERTED = false;
        public static final boolean IS_AUX_INVERTED = false;
        public static final boolean IS_MAIN_SENSOR_INVERTED = false;
        public static final boolean IS_AUX_SENSOR_INVERTED = false;
    }

    public static class Shooter {
        public static final int MOTOR = 0; // Main motor port.
        public static final TalonFXConfiguration CONFIGURATION = new TalonFXConfiguration();
    }

    public static class Controls {
        public static final int XBOX = 0; // Xbox controller port.
    }

    public static class Leds{
        public static final int SHOOTER_LED_PORT = 0;
    }
}
