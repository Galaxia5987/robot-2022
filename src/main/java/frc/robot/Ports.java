package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

public final class Ports {

    public static final class SwerveDrive {
        // front right
        public static final int DRIVE_MOTOR_FR = 1;
        public static final int ANGLE_MOTOR_FR = 2;
        public static final boolean DRIVE_INVERTED_FR = false;
        public static final boolean ANGLE_INVERTED_FR = false;
        public static final boolean ANGLE_SENSOR_PHASE_FR = true;

        // front left
        public static final int DRIVE_MOTOR_FL = 5;
        public static final int ANGLE_MOTOR_FL = 6;
        public static final boolean DRIVE_INVERTED_FL = false;
        public static final boolean ANGLE_INVERTED_FL = true;
        public static final boolean ANGLE_SENSOR_PHASE_FL = false;

        // rear right
        public static final int DRIVE_MOTOR_RR = 3;
        public static final int ANGLE_MOTOR_RR = 4;
        public static final boolean DRIVE_INVERTED_RR = false;
        public static final boolean ANGLE_INVERTED_RR = true;
        public static final boolean ANGLE_SENSOR_PHASE_RR = false;

        // rear left
        public static final int DRIVE_MOTOR_RL = 6;
        public static final int ANGLE_MOTOR_RL = 7;
        public static final boolean DRIVE_INVERTED_RL = false;
        public static final boolean ANGLE_INVERTED_RL = true;
        public static final boolean ANGLE_SENSOR_PHASE_RL = false;
    }

    public static class Controls {
        public static final int XBOX = 0;
        public static final int JOYSTICK = 2;
        public static final int JOYSTICK2 = 3;
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

    public static class Vision {
        public static final int LEDS = 3;
    }
}
