package frc.robot;

import frc.robot.valuetuner.WebConstant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.SimVisionTarget;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


public final class Constants {
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]

    public static final double LOOP_PERIOD = 0.02; // loop period. [s]
    public static final double g = 9.80665; // Gravity acceleration constant. [m/s^2]
    public static final double UPPER_TARGET_HEIGHT = 2.64; // Height of upper target. [m]
    public static final double PEBZNER_HEIGHT = 4.8; // Height of pebzner auditorium. [m]
    public static final double NOMINAL_VOLTAGE = 12.0; // Nominal voltage. [V]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations. [ms]


    public static class Shooter {
        public static final double TICKS_PER_REVOLUTION = 2048; // Ticks per revolution of the shooter motor. [tick]
        public static final double WHEEL_RADIUS = 0.1016; // Radius of the wheels. [m]

        public static final double Ka = 1; // Acceleration state space coefficient (placeholder).
        public static final double Kv = 1; // Velocity state space coefficient (placeholder).
        public static final double MODEL_TOLERANCE = 10; // Model tolerance for state space.
        public static final double SENSOR_TOLERANCE = 0.1; // Sensor tolerance for state space.
        public static final double VELOCITY_TOLERANCE = 0.15; // Velocity tolerance for state space.
        public static final double COST_LQR = 55; // Cost lqr for state space.

        public static final double J = 0.00218; // Moment of inertia for state space. [kg*m^2]
        public static final double GEAR_RATIO = 1; // Gear ratio for encoder (placeholder).
        public static final double NEUTRAL_DEADBAND = 0.1; // [%]

        public static final double OUTPUT_MULTIPLIER = 0.1; // Multiplies the output for manual control in the bits. [%]

        public static TalonFXConfiguration getConfiguration() {
            final TalonFXConfiguration configuration = new TalonFXConfiguration();
            configuration.neutralDeadband = NEUTRAL_DEADBAND;
            return configuration;
        }
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

    public static class Intake {
        public static final double DEFAULT_POWER = 0.5; // power intake will receive on the basic command. [%]
        public static final double POWER_TO_VELOCITY_RATIO = -3 / 16.0; // Ratio of power to velocity. [% / m/s]
        public static final boolean IS_COMPENSATING_VOLTAGE = true;

    public static class Control {
        public static final double RIGHT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
    }
}
