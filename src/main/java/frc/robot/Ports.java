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

    public static class Controls {
        public static final int JOYSTICK = 0;
        public static final int XBOX = 0;
    }

    /*
     left motor's data, right motor's data.
     */
    public static class Climber {
        public static final int MAIN = 41;
        public static final int AUX = 42;
        public static final int STOPPER = 7;

        public static final boolean SENSOR_PHASE = true;

        public static final TalonFXInvertType IS_AUX_INVERTED = TalonFXInvertType.Clockwise;
        public static final TalonFXInvertType IS_MAIN_INVERTED = TalonFXInvertType.Clockwise;

        /*
         Used only for simulation.
         */
        public static final int ENCODER_A_CHANNEL = 0;
        public static final int ENCODER_B_CHANNEL = 1;
    }
}