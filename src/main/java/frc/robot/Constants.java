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
import frc.robot.valuetuner.WebConstant;
import org.photonvision.SimVisionTarget;

import java.util.HashMap;

import static frc.robot.Ports.SwerveDrive.*;


public final class Constants {
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].

    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final double NOMINAL_VOLTAGE = 12; // [volts]
    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]
    public static final double SIMULATION_LOOP_PERIOD = 0.02; // [s]

    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    // The order of modules is ALWAYS front-right (fr), front-left (fl), rear-right (rr), rear-left (rl)
    public static final class SwerveDrive {
        public static final double VELOCITY_MULTIPLIER = 3;
        public static final double ROTATION_MULTIPLIER = 3;

        public static final int TICKS_PER_ROTATION_DRIVE_MOTOR = 2048;
        public static final int TICKS_PER_ROTATION_ANGLE_MOTOR = 1024;
        public static final double GEAR_RATIO_DRIVE_MOTOR = 7.5;
        public static final double GEAR_RATIO_ANGLE_MOTOR = 1;
        public static final double DRIVE_MOTOR_TICKS_PER_METER = GEAR_RATIO_DRIVE_MOTOR * TICKS_PER_ROTATION_DRIVE_MOTOR / (0.11 * Math.PI); // 4 * 0.0254
        public static final double ANGLE_MOTOR_TICKS_PER_RADIAN = GEAR_RATIO_ANGLE_MOTOR * TICKS_PER_ROTATION_ANGLE_MOTOR / (2 * Math.PI);

        public static final int MAX_CURRENT = 15; // [amps]

        // State Space
        public static final double VELOCITY_TOLERANCE = 5; // [rps]
        public static final double COST_LQR = 11;
        // Note that the values of MODEL_TOLERANCE and ENCODER_TOLERANCE should be a lot smaller (something like 1e-6)
        public static final double MODEL_TOLERANCE = 0.01;
        public static final double ENCODER_TOLERANCE = 0.01; // [ticks]

        public static final double HEADING_KP = 5;
        public static final double HEADING_KI = 0;
        public static final double HEADING_KD = 0;
        public static final TrapezoidProfile.Constraints HEADING_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(10, 5); // [rads/sec], [rad/sec^2]

        // The heading is responsible for the angle of the whole chassis, while the angle is used in the angle motor itself.
        public static final double ALLOWABLE_HEADING_ERROR = Math.toRadians(5); // [rad]
        public static final double ALLOWABLE_ANGLE_ERROR = Math.toRadians(3); // [rad]
        public static final double WHEEL_RADIUS = 0.04688; // [m]

        public static final double ROBOT_LENGTH = 0.6624; // [m]
        public static final double ROBOT_WIDTH = 0.5224; // [m]

        // the rotational velocity of the robot, this constant multiplies the rotation output of the joystick
        public static final int ANGLE_CURVE_STRENGTH = 1;
        public static final int ANGLE_CRUISE_VELOCITY = 400;
        public static final int ANGLE_MOTION_ACCELERATION = 1300;

        public static final double JOYSTICK_THRESHOLD = 0.05; // [%]
        public static final double ROTATIONAL_ADDITION_RESTRAINT = 3;
        public static final double DRIFTING_PERIOD = 0.5; // expected period the robot will change its rotation even after commanded to stop. [s]
        public static final double SAMPLE_YAW_PERIOD = 0.1; // expected period the robot will change its rotation even after commanded to stop. [s]
        public static final double ADJUST_CONTROLLER_TOLERANCE = Math.toRadians(0.5);
        public static final WebConstant ADJUST_CONTROLLER_KP = WebConstant.of("Swerve", "kp_heading", 12.5);
        public static final WebConstant ADJUST_CONTROLLER_KI = WebConstant.of("Swerve", "KI_heading", 0);
        public static final WebConstant ADJUST_CONTROLLER_KD = WebConstant.of("Swerve", "Kd_heading", 0.1);

        private static final double Rx = SwerveDrive.ROBOT_LENGTH / 2; // [m]
        private static final double Ry = SwerveDrive.ROBOT_WIDTH / 2; // [m]
        
        // Axis systems
        public static final Translation2d[] SWERVE_POSITIONS = new Translation2d[]{
                new Translation2d(Rx, -Ry),
                new Translation2d(Rx, Ry),
                new Translation2d(-Rx, -Ry),
                new Translation2d(-Rx, Ry)
        };
    }

    public static class Shooter {
        public static final double TICKS_PER_REVOLUTION = 2048; // Ticks per revolution of the shooter motor. [tick]
        public static final double NEUTRAL_DEADBAND = 0.1; // [%]
        public static final double OUTPUT_MULTIPLIER = 0.1; // Multiplies the output for manual control in the bits. [%]
        public static final double OUTTAKE_POWER = 0.2; // Power to give to the shooter when taking balls out. [%]
        public static final double RECOMMENDED_ACCELERATION_TIME = 1.3; // Recommended time for the shooter to get to it's setpoint. [s]
        public static final double CARGO_OFFSET = 0; // Desired offset from the middle of the target where you want the cargo to hit. [m]

        public static final WebConstant kP = WebConstant.of("Shooter", "kP", 0.035);
        public static final WebConstant kI = WebConstant.of("Shooter", "kI", 0);
        public static final WebConstant kD = WebConstant.of("Shooter", "kD", 2);
        public static final WebConstant kF = WebConstant.of("Shooter", "kf", 0.047);
        public static final WebConstant SHOOTER_VELOCITY_DEADBAND = WebConstant.of("Shooter", "Velocity deadband", 50); // Dead band for shooter velocity setpoint. [rpm]

        public static final HashMap<Double, Double> SHORT_MEASUREMENTS = new HashMap<>() {{
            put(-99999.0, 3350.0);
            put(1.9, 3350.0);
            put(2.15, 3500.0);
            put(2.39, 3800.0);
            put(2.67, 3925.0);
            put(99999.0, 3925.0);
        }};
        public static final HashMap<Double, Double> LONG_MEASUREMENTS = new HashMap<>() {{
            put(-99999.0, 3350.0);
            put(2.67, 3350.0);
            put(2.89, 3500.0);
            put(3.04, 3550.0);
            put(3.16, 3600.0);
            put(3.29, 3640.0);
            put(3.42, 3665.0);
            put(3.6, 3700.0);
            put(3.73, 3750.0);
            put(3.87, 3850.0);
            put(4.02, 3950.0);
            put(4.25, 3880.0);
            put(4.41, 4015.0);
            put(4.61, 4165.0);
            put(4.82, 4219.0);
            put(5.06, 4250.0);
            put(5.23, 4325.0);
            put(5.37, 4400.0);
            put(5.65, 4550.0);
            put(99999.0, 4550.0);
        }};

        public static TalonFXConfiguration getConfiguration() {
            final TalonFXConfiguration configuration = new TalonFXConfiguration();
            configuration.neutralDeadband = NEUTRAL_DEADBAND;
            return configuration;
        }
    }

    public static final class SwerveModule {
        public static final int TRIGGER_THRESHOLD_CURRENT = 2; // [amps]

        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // [secs]
        public static final double RAMP_RATE = 0; // seconds from neutral to max

        public static final int[] ZERO_POSITIONS = {-635, -103, -1054, -434}; // fr, fl, rr, rl

        public static final SwerveModuleConfigBase frConfig = new SwerveModuleConfigBase.Builder(0)
                .configPorts(DRIVE_MOTOR_FR, ANGLE_MOTOR_FR)
                .configInversions(DRIVE_INVERTED_FR, ANGLE_INVERTED_FR, ANGLE_SENSOR_PHASE_FR)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[0])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase flConfig = new SwerveModuleConfigBase.Builder(1)
                .configPorts(DRIVE_MOTOR_FL, ANGLE_MOTOR_FL)
                .configInversions(DRIVE_INVERTED_FL, ANGLE_INVERTED_FL, ANGLE_SENSOR_PHASE_FL)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[1])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rrConfig = new SwerveModuleConfigBase.Builder(2)
                .configPorts(DRIVE_MOTOR_RR, ANGLE_MOTOR_RR)
                .configInversions(DRIVE_INVERTED_RR, ANGLE_INVERTED_RR, ANGLE_SENSOR_PHASE_RR)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[2])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rlConfig = new SwerveModuleConfigBase.Builder(3)
                .configPorts(DRIVE_MOTOR_RL, ANGLE_MOTOR_RL)
                .configInversions(DRIVE_INVERTED_RL, ANGLE_INVERTED_RL, ANGLE_SENSOR_PHASE_RL)
                .configAnglePID(6, 0, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[3])
                .configJ(0.115)
                .build();
    }

    public static class Autonomous {
        public static final double KP_THETA_CONTROLLER = 8.8;
        public static final double KP_X_CONTROLLER = 3;
        public static final double KP_Y_CONTROLLER = 3;

        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0, 0, 0);
        public static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(0);
        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

        public static final double MAX_VEL = 4; // [m/sec] 4.1
        public static final double MAX_ACCEL = 2; // [m/sec^2] 2.14
    }

    public static class Conveyor {
        public static final int MIN_PROXIMITY_VALUE = 200; // Minimum distance from the color sensor in order to induce detection (arbitrary bit units).

        public static final double TICKS_PER_UNIT = 2048 * 2;
        public static final double SHOOT_POWER = 0.5;

        public static final Color BLUE = new Color(0.18, 0.385, 0.436);
        public static final Color RED = new Color(0.511, 0.346, 0.143);
        public static final Color NONE = new Color(0.31, 0.415, 0.275);

        public static final WebConstant DEFAULT_POWER = WebConstant.of("Conveyor", "Default velocity", 0.5); // [%]
    }

    public static class Vision { //TODO: change for competition
        public static final int CAM_RESOLUTION_HEIGHT = 480; // Height of camera resolution. [pixel]
        public static final int CAM_RESOLUTION_WIDTH = 640; // Width of camera resolution. [pixel]

        public static final double CAMERA_HEIGHT = 0.73; // [m]
        public static final double TARGET_HEIGHT_FROM_GROUND = 2.65; // [m] Pefzener 2.62
        public static final double CAMERA_PITCH = 36.2; // Pitch of the vision. [deg]
        public static final double DIAG_FOV = 75; // Diagonal FOV. [deg]
        public static final double LED_RANGE = 6; // Visible range of LEDs. [m]
        public static final double MIN_TARGET_AREA = 10; // Minimal area of target. [pixel^2]
        public static final double TARGET_WIDTH = 1.36; // Width of vision target strip. [m]
        public static final double TARGET_HEIGHT = 0.05; // Height of the vision target strip. [m]
        public static final double TARGET_RADIUS = 0.678;

        public static final Pose2d HUB_POSE = new Pose2d( // Position of the hub relative to the field.
                new Translation2d(FIELD_LENGTH / 2, FIELD_WIDTH / 2), new Rotation2d());
        public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(
                new Translation2d(0.038, 0.171), new Rotation2d()); // Position of the vision relative to the robot.

        public static final SimVisionTarget SIM_TARGET_HUB = new SimVisionTarget( // Hub target for vision simulation.
                HUB_POSE, TARGET_HEIGHT_FROM_GROUND, TARGET_WIDTH, TARGET_HEIGHT);

    }

    public static class Helicopter {
        public static final double KP = 0.068;
        public static final double KI = 0;
        public static final double KD = 0;

        public static final double CRUISE_VELOCITY = 0; // [ticks/100ms]
        public static final double MAXIMAL_ACCELERATION = 0; // [ticks/100ms*sec]

        public static final double MAX_VELOCITY = Math.PI * 2 / 3; // [rad/s]

        public static final double GEAR_RATIO = 292.1;

        public static final double TICKS_PER_RAD = 2048 * GEAR_RATIO / (2 * Math.PI);
        public static final double TICKS_PER_RAD_ABSOLUTE_ENCODER = 1 / (2 * Math.PI);

        public static final double JOYSTICK_DEADBAND = 0.05; // [%]

        public static final double ARM_ENCODER_DIST_PER_PULSE = 2.0 * Math.PI / 2048;
        public static final double ARM_MASS = 5.0; // Kilograms
        public static final double ARM_LENGTH = 0.792; // [m]

        public static final double MAX_ANGLE = Math.toRadians(255); // [radians]
        public static final double MIN_ANGLE = Math.toRadians(-75); // [radians]

        public static final double STOP_HELICOPTER_TIMESTAMP = 149.5; // [s]

        public static final double ZERO_POSITION = -0.050759776268994417; // [ticks]

        public static final double POSITION_TOLERANCE = 0.015; // [radians]

        public static final double SECOND_RUNG = Math.toRadians(110.66); // [radians]
        public static final double THIRD_RUNG = 0; // [radians]
        public static final double RUNG_SEPARATION = 0; // [radians]

        public static final double BLUE_RUNG_YAW = 180;
        public static final double RED_RUNG_YAW = BLUE_RUNG_YAW - 180;

        public static final double OSCILLATION_DELTA_TIME = 0.2;

        public static final boolean VOLTAGE_COMPENSATION = true;
    }

    public static class Intake {
        public static final double POWER_TO_VELOCITY_RATIO = -3 / 16.0; // Ratio of power to velocity. [% / m/s]
        public static final double TIME_BETWEEN_RUNS = 1.7; // time intake will wait before toggling the retractor (for testing only). [s]

        public static final WebConstant DEFAULT_POWER = WebConstant.of("Intake", "power", 1); // power intake will receive on the basic command. [%]
    }

    public static class Hood {
        public static final double HOOD_PRESSURE_BIT_DELTA_TIME = 0.1; // [s]
        public static final double DISTANCE_FROM_TARGET_THRESHOLD = 2.67; // [m]
    }

    public static class Control {
        public static final int JOYSTICK_FILTER_TAP = 8 + 8 / 2;

        public static final double RIGHT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
        public static final double LEFT_TRIGGER_DEADBAND = 0.4; // Deadband for right trigger. [%]
    }

    public static class Flap {
        public static final double OSCILLATION_DELTA_TIME = 0.4; // [sec]
    }
}
