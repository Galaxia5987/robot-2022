package frc.robot;

import frc.robot.valuetuner.WebConstant;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.SwerveModuleConfigBase;

import static frc.robot.Ports.SwerveDrive.*;

public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // [s]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].
    public static final double NOMINAL_VOLTAGE = 12; // [volts]
    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    // The order of modules is ALWAYS front-right (fr), front-left (fl), rear-right (rr), rear-left (rl)
    public static final class SwerveDrive {

        public static final double VELOCITY_MULTIPLIER = 2.5;
        public static final double ROTATION_MULTIPLIER = 5;

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

        public static final double THETA_KP = 1;
        public static final double THETA_KI = 1;
        public static final double THETA_KD = 1;

        // The theta is responsible for the angle of the whole chassis, while the angle is used in the angle motor itself.
        public static final double ALLOWABLE_THETA_ERROR = Math.toRadians(0.05); // [rad]
        public static final double ALLOWABLE_ANGLE_ERROR = Math.toRadians(8); // [rad]
        public static final double WHEEL_RADIUS = 0.04688; // [m]

        public static final double ROBOT_LENGTH = 0.5924; // [m]
        public static final double ROBOT_WIDTH = 0.5924; // [m]
        // the rotational velocity of the robot, this constant multiplies the rotation output of the joystick
        public static final double JOYSTICK_THRESHOLD = 0.1; // [%]
        public static final double OUTER_JOYSTICK_THRESHOLD = 0.95; // [%]
        public static final double JOYSTICK_ANGLE_DEADZONE = 5; // [degrees]
        public static final double ROTATION_DELAY = 0.1; // [sec]
        public static final int ANGLE_CURVE_STRENGTH = 4;
        private static final double Rx = SwerveDrive.ROBOT_WIDTH / 2;
        private static final double Ry = SwerveDrive.ROBOT_LENGTH / 2;
        // angle motion magic
        private static final float MOTION_MAGIC_SAFETY = 0.7f;
        public static final int ANGLE_MOTION_ACCELERATION = Math.round(2800 * MOTION_MAGIC_SAFETY);
        public static final int ANGLE_CRUISE_VELOCITY = Math.round(550 * MOTION_MAGIC_SAFETY);
        // Axis systems
        public static final Translation2d[] SWERVE_POSITIONS = new Translation2d[]{
                new Translation2d(Rx, -Ry),
                new Translation2d(Rx, Ry),
                new Translation2d(-Rx, -Ry),
                new Translation2d(-Rx, Ry)
        };
    }

    public static final class SwerveModule {
        public static final int[] ZERO_POSITIONS = {7195, -503, -1329, 94}; // fr, fl, rr, rl

        public static final int TRIGGER_THRESHOLD_CURRENT = 2; // [amps]
        public static final double TRIGGER_THRESHOLD_TIME = 0.02; // [secs]
        public static final double RAMP_RATE = 1; // seconds from neutral to max

        public static final SwerveModuleConfigBase frConfig = new SwerveModuleConfigBase.Builder(0).configPorts(DRIVE_MOTOR_FR, ANGLE_MOTOR_FR)
                .configInversions(DRIVE_INVERTED_FR, ANGLE_INVERTED_FR, ANGLE_SENSOR_PHASE_FR)
                .configAnglePID(4.5, 0.0045, 1, 0)
                .configZeroPosition(ZERO_POSITIONS[0])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase flConfig = new SwerveModuleConfigBase.Builder(1).configPorts(DRIVE_MOTOR_FL, ANGLE_MOTOR_FL)
                .configInversions(DRIVE_INVERTED_FL, ANGLE_INVERTED_FL, ANGLE_SENSOR_PHASE_FL)
                .configAnglePID(13, 0.0045, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[1])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rrConfig = new SwerveModuleConfigBase.Builder(2).configPorts(DRIVE_MOTOR_RR, ANGLE_MOTOR_RR)
                .configInversions(DRIVE_INVERTED_RR, ANGLE_INVERTED_RR, ANGLE_SENSOR_PHASE_RR)
                .configAnglePID(8, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[2])
                .configJ(0.115)
                .build();

        public static final SwerveModuleConfigBase rlConfig = new SwerveModuleConfigBase.Builder(3).configPorts(DRIVE_MOTOR_RL, ANGLE_MOTOR_RL)
                .configInversions(DRIVE_INVERTED_RL, ANGLE_INVERTED_RL, ANGLE_SENSOR_PHASE_RL)
                .configAnglePID(10, 0.004, 0, 0)
                .configZeroPosition(ZERO_POSITIONS[3])
                .configJ(0.115)
                .build();
    }

    public static class Autonomous {
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(3, 1.5); // [rads/sec]
        public static final double kPThetaController = 2;
        public static final double kPXController = 2;
        public static final double kPYController = 2;

        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0, 0, 0);
        public static final Matrix<N1, N1> localMeasurementStdDevs = VecBuilder.fill(0);
        public static final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0, 0, 0);

        public static final double MAX_VEL = 3;
        public static final double MAX_ACCEL = 1.5;
    }

    public static final double FIELD_WIDTH = 8.23; // Width of the field. [m]
    public static final double FIELD_LENGTH = 16.46; // Length of the field. [m]

    public static class ExampleSubsystem {
        private static final String NAME = ExampleSubsystem.class.getName();
        public static final WebConstant POWER = WebConstant.of(NAME, "power", 0.5); // [%]
    }
}
