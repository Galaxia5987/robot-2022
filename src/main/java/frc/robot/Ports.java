package frc.robot;

public final class Ports {

    public static final class SwerveDrive {
        // front right
        public static final int DRIVE_MOTOR_FR = 23;
        public static final int ANGLE_MOTOR_FR = 24;
        public static final boolean DRIVE_INVERTED_FR = false;
        public static final boolean ANGLE_INVERTED_FR = false;
        public static final boolean ANGLE_SENSOR_PHASE_FR = true;

        // front left
        public static final int DRIVE_MOTOR_FL = 27;
        public static final int ANGLE_MOTOR_FL = 28;
        public static final boolean DRIVE_INVERTED_FL = false;
        public static final boolean ANGLE_INVERTED_FL = true;
        public static final boolean ANGLE_SENSOR_PHASE_FL = false;

        // rear right
        public static final int DRIVE_MOTOR_RR = 21;
        public static final int ANGLE_MOTOR_RR = 22;
        public static final boolean DRIVE_INVERTED_RR = false;
        public static final boolean ANGLE_INVERTED_RR = true;
        public static final boolean ANGLE_SENSOR_PHASE_RR = false;

        // rear left
        public static final int DRIVE_MOTOR_RL = 25;
        public static final int ANGLE_MOTOR_RL = 26;
        public static final boolean DRIVE_INVERTED_RL = false;
        public static final boolean ANGLE_INVERTED_RL = true;
        public static final boolean ANGLE_SENSOR_PHASE_RL = false;
    }

    public static class Controls {
        public static final int XBOX = 0;
        public static final int JOYSTICK = 2;
        public static final int JOYSTICK2 = 3;
    }
}
