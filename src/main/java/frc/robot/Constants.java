package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.SwerveModuleConfigBase;
import org.photonvision.SimVisionTarget;

import static frc.robot.Ports.SwerveDrive.*;


public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].
    public static final double NOMINAL_VOLTAGE = 12; // [volts]
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]
    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final double SIMULATION_LOOP_PERIOD = 0.02; // [s]

    // The order of modules is ALWAYS front-right (fr), front-left (fl), rear-right (rr), rear-left (rl)
    public static final class SwerveDrive {

        public static final double VELOCITY_MULTIPLIER = 2.2;
        public static final double ROTATION_MULTIPLIER = 2;

        public static final int TICKS_PER_ROTATION_DRIVE_MOTOR = 2048;
        public static final int TICKS_PER_ROTATION_ANGLE_MOTOR = 1024;
        public static final double GEAR_RATIO_DRIVE_MOTOR = 7.5;
        public static final double GEAR_RATIO_ANGLE_MOTOR = 1;
        public static final double DRIVE_MOTOR_TICKS_PER_METER = GEAR_RATIO_DRIVE_MOTOR * TICKS_PER_ROTATION_DRIVE_MOTOR / (4 * 0.0254 * Math.PI);
        public static final double ANGLE_MOTOR_TICKS_PER_RADIAN = GEAR_RATIO_ANGLE_MOTOR * TICKS_PER_ROTATION_ANGLE_MOTOR / (2 * Math.PI);

        public static final int MAX_CURRENT = 15; // [amps]

        // State Space
        public static final double VELOCITY_TOLERANCE = 5; // [rps]
        public static final double COST_LQR = 11;
        // Note that the values of MODEL_TOLERANCE and ENCODER_TOLERANCE should be a lot smaller (something like 1e-6)
        public static final double MODEL_TOLERANCE = 0.01;
        public static final double ENCODER_TOLERANCE = 0.01; // [ticks]

        public static final double HEADING_KP = 1;
        public static final double HEADING_KI = 1;
        public static final double HEADING_KD = 1;
        public static final TrapezoidProfile.Constraints HEADING_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 1.5); // [rads/sec], [rad/sec^2]

        // The heading is responsible for the angle of the whole chassis, while the angle is used in the angle motor itself.
        public static final double ALLOWABLE_HEADING_ERROR = Math.toRadians(0.05); // [rad]
        public static final double ALLOWABLE_ANGLE_ERROR = Math.toRadians(8); // [rad]
        public static final double WHEEL_RADIUS = 0.04688; // [m]

        public static final double ROBOT_LENGTH = 0.5924; // [m]
        public static final double ROBOT_WIDTH = 0.5924; // [m]

        // the rotational velocity of the robot, this constant multiplies the rotation output of the joystick
        public static final double JOYSTICK_THRESHOLD = 0.1; // [%]
        public static final double ANGLE_COSINE_DEADBAND = Math.toRadians(10); // [rads]
        public static final double ROTATION_DELAY = 0.1; // [sec]
        public static final int ANGLE_CURVE_STRENGTH = 4;
        public static final double LOOP_PERIOD = 0.02; // loop period. [s]
        public static final double g = 9.80665; // Gravity acceleration constant. [m/s^2]
        public static final double UPPER_TARGET_HEIGHT = 2.64; // Height of upper target. [m]
        public static final double PEBZNER_HEIGHT = 4.8; // Height of pebzner auditorium. [m]
        public static final double NOMINAL_VOLTAGE = 12.0; // Nominal voltage. [V]
        public static final int TALON_TIMEOUT = 10; // Waiting period for configurations. [ms]
        private static final double Rx = SwerveDrive.ROBOT_WIDTH / 2; // [m]
        private static final double Ry = SwerveDrive.ROBOT_LENGTH / 2; // [m]
        // Axis systems
        public static final Translation2d[] SWERVE_POSITIONS = new Translation2d[]{
                new Translation2d(Rx, -Ry),
                new Translation2d(Rx, Ry),
                new Translation2d(-Rx, -Ry),
                new Translation2d(-Rx, Ry)
        };
        // angle motion magic
        private static final float MOTION_MAGIC_SAFETY = 0.7f;
        public static final int ANGLE_MOTION_ACCELERATION = Math.round(2800 * MOTION_MAGIC_SAFETY);
        public static final int ANGLE_CRUISE_VELOCITY = Math.round(550 * MOTION_MAGIC_SAFETY);
    }

    public static class Shooter {
        public static final double TICKS_PER_REVOLUTION = 2048; // Ticks per revolution of the shooter motor. [tick]
        public static final double WHEEL_RADIUS = 0.1016; // Radius of the wheels. [m]

        public static final double Ka = 1; // Acceleration state space coefficient (placeholder).
        public static final double Kv = 1; // Velocity state space coefficient (placeholder).
        public static final double MODEL_TOLERANCE = 1e6; // Model tolerance for state space.
        public static final double SENSOR_TOLERANCE = 1e-6; // Sensor tolerance for state space.
        public static final double QELMS = 1; // Velocity tolerance for state space.
        public static final double RELMS = 10; // Cost lqr for state space.

        public static final double J = 0.00718; // Moment of inertia for state space. [kg*m^2]
        public static final double GEAR_RATIO = 1; // Gear ratio for encoder (placeholder).
        public static final double NEUTRAL_DEADBAND = 0.1; // [%]

        public static final double OUTPUT_MULTIPLIER = 0.1; // Multiplies the output for manual control in the bits. [%]
        public static final double OUTTAKE_POWER = 0.2; // Power to give to the shooter when taking balls out. [%]
        public static final double SHOOTER_VELOCITY_DEADBAND = 1 - 2.95 / 3.0; // Dead band for shooter velocity setpoint. [%]
        public static double RECOMMENDED_ACCELERATION_TIME = 1.3; // Recommended time for the shooter to get to it's setpoint. [s]

        public static TalonFXConfiguration getConfiguration() {
            final TalonFXConfiguration configuration = new TalonFXConfiguration();
            configuration.neutralDeadband = NEUTRAL_DEADBAND;
            return configuration;
        }

    }

    public static final class SwerveModule {
        public static final int[] ZERO_POSITIONS = {-3697, 538, -1813, -1950}; // fr, fl, rr, rl

        public static final int TRIGGER_THRESHOLD_CURRENT = 2; // [amps]
        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // [secs]
        public static final double RAMP_RATE = 1; // seconds from neutral to max

        public static final SwerveModuleConfigBase frConfig = new SwerveModuleConfigBase.Builder(0)
                .configPorts(DRIVE_MOTOR_FR, ANGLE_MOTOR_FR)
                .configInversions(DRIVE_INVERTED_FR, ANGLE_INVERTED_FR, ANGLE_SENSOR_PHASE_FR)
                .configAnglePID(4.5, 0.0045, 1, 0)
                .configZeroPosition(ZERO_POSITIONS[0])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase flConfig = new SwerveModuleConfigBase.Builder(1)
                .configPorts(DRIVE_MOTOR_FL, ANGLE_MOTOR_FL)
                .configInversions(DRIVE_INVERTED_FL, ANGLE_INVERTED_FL, ANGLE_SENSOR_PHASE_FL)
                .configAnglePID(13, 0.0045, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[1])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rrConfig = new SwerveModuleConfigBase.Builder(2)
                .configPorts(DRIVE_MOTOR_RR, ANGLE_MOTOR_RR)
                .configInversions(DRIVE_INVERTED_RR, ANGLE_INVERTED_RR, ANGLE_SENSOR_PHASE_RR)
                .configAnglePID(8, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[2])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rlConfig = new SwerveModuleConfigBase.Builder(3)
                .configPorts(DRIVE_MOTOR_RL, ANGLE_MOTOR_RL)
                .configInversions(DRIVE_INVERTED_RL, ANGLE_INVERTED_RL, ANGLE_SENSOR_PHASE_RL)
                .configAnglePID(10, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[3])
                .configJ(0.115)
                .build();
    }

    public static class Autonomous {
        public static final double KP_THETA_CONTROLLER = 2;
        public static final double KP_X_CONTROLLER = 2;
        public static final double KP_Y_CONTROLLER = 2;

        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0, 0, 0);
        public static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(0);
        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

        public static final double MAX_VEL = 3; // [m/sec]
        public static final double MAX_ACCEL = 1.5; // [m/sec^2]
    }

    public static class Conveyor {
        public static final double DEFAULT_POWER = 0.5; // [%]
        public static final int MAX_CARGO_AMOUNT = 2;
        public static final int MIN_PROXIMITY_VALUE = 100; // Minimum distance from the color sensor in order to induce detection (arbitrary bit units).
        public static final Color RED = new Color(0.19, 0.49, 0.32);
        public static final Color BLUE = new Color(2.4e-4, 0.51, 0.487);
        public static final Color NONE = new Color(2.4e-4, 0.52, 0.4);

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

    public static class Helicopter {
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

        public static final double MAX_VELOCITY = Math.PI * 2 / 3; // [rad/s]

        public static final double GEAR_RATIO = 292.1;

        public static final double TICKS_PER_RAD = 2048 * GEAR_RATIO / (2 * Math.PI);

        public static final double JOYSTICK_DEADBAND = 0.05; // [%]

        public static final double ARM_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 2048;
        public static final double ARM_MASS = 5.0; // Kilograms
        public static final double ARM_LENGTH = 0.792; // [m]

        public static final double MAX_ANGLE = Math.toRadians(255); // [radians]
        public static final double MIN_ANGLE = Math.toRadians(-75); // [radians]

        public static final double STOP_HELICOPTER_TIMESTAMP = 149.5; // [s]

        public static final int ZERO_POSITION = 0; // [radians]

        public static final double ZERO_POSITION_TOLERANCE = 0.05; // [radians]
    }

    public static class Intake {
        public static final double DEFAULT_POWER = 1; // power intake will receive on the basic command. [%]
        public static final double POWER_TO_VELOCITY_RATIO = -3 / 16.0; // Ratio of power to velocity. [% / m/s]
        public static final boolean IS_COMPENSATING_VOLTAGE = true;
        public static final double TIME_BETWEEN_RUNS = 1.0; // time intake will wait before toggling the retractor (for testing only). [s]
    }

    public static class Hood {
        public static final double HOOD_PRESSURE_BIT_DELTA_TIME = 0.1; // [s]
        public static final double DISTANCE_FROM_TARGET_THRESHOLD = 3.5; // [m]
    }

    public static class Control {
        public static final double RIGHT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
        public static final double LEFT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]

    }
}
