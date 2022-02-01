package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

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
        public static final TalonFXInvertType INVERSION_TYPE = TalonFXInvertType.CounterClockwise;

    public static class Hood {
        public static final int SOLENOID = 0; // Hood solenoid port.
    }

    public static class Controls {
        public static final int XBOX = 0; // Xbox controller port.
    }

    public static class Intake {
        public static final int MOTOR = 0;
        public static final boolean IS_MOTOR_INVERTED = false;
        public static final int SOLENOID = 0;
    }
}
