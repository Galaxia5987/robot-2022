package frc.robot;

import frc.robot.valuetuner.WebConstant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.SimVisionTarget;

public final class Constants {
    public static final double NOMINAL_VOLTAGE = 12; // [volts]
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]

    public static class ExampleSubsystem {
        private static final String NAME = ExampleSubsystem.class.getName();
        public static final WebConstant POWER = WebConstant.of(NAME, "power", 0.5); // [%]
    }

    public static class Climber {
        public static final double KP = 1.5;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double F_FORWARD_S = 0;
        public static final double F_FORWARD_COS = 0;
        public static final double F_FORWARD_V = 0;
        public static final double F_FORWARD_A = 0;

        public static final double CRUISE_VELOCITY = 0; // [ticks/100ms]
        public static final double MAXIMAL_ACCELERATION = 0; // [ticks/100ms*sec]
        public static final boolean VOLTAGE_COMPENSATION = true;

        public static final double MAX_VELOCITY = Math.PI * 2 / 3; //[rad/s]

        public static final double GEAR_RATIO = 292.1;

        public static final double TICKS_PER_RAD = 2048 * GEAR_RATIO / (2 * Math.PI);

        public static final double JOYSTICK_DEADBAND = 0.05; // [%]

        public static final double ARM_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 2048;
        public static final double ARM_MASS = 5.0; // Kilograms
        public static final double ARM_LENGTH = 0.792; // [m]

        public static final double MAX_ANGLE = Math.toRadians(255); // [radians]
        public static final double MIN_ANGLE = Math.toRadians(-75); // [radians]

        public static final double STOP_CLIMBER = 149.5; // [s]
    }

    public static class Vision {
        public static final double CAMERA_HEIGHT = 0.767; // [m]
        public static final double TARGET_HEIGHT_FROM_GROUND = 2.64; // [m]
        public static final double CAMERA_PITCH = 30; // Pitch of the vision. [deg]
        public static final double DIAG_FOV = 75; // Diagonal FOV. [deg]
        public static final double LED_RANGE = 6; // Visible range of LEDs. [m]
        public static final int CAM_RESOLUTION_WIDTH = 640; // Width of camera resolution. [pixel]
        public static final int CAM_RESOLUTION_HEIGHT = 480; // Height of camera resolution. [pixel]
        public static final double MIN_TARGET_AREA = 10; // Minimal area of target. [pixel^2]
        public static final double TARGET_WIDTH = 1.36; // Width of vision target strip. [m]
        public static final double TARGET_HEIGHT = 0.05; // Height of the vision target strip. [m]

        public static final Pose2d HUB_POSE = new Pose2d( // Position of the hub relative to the field.
                new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2), new Rotation2d());
        public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(
                new Translation2d(0.038, 0.171), new Rotation2d()); // Position of the vision relative to the robot.

        public static final SimVisionTarget SIM_TARGET_HUB = new SimVisionTarget( // Hub target for vision simulation.
                HUB_POSE, TARGET_HEIGHT_FROM_GROUND, TARGET_WIDTH, TARGET_HEIGHT);
    }
}
