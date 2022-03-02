package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.TimeDelayedBoolean;
import frc.robot.utils.VisionEstimationData;
import webapp.FireLog;

import java.util.function.Supplier;

/**
 * The {@code SwerveDrive} Subsystem is responsible for the integration of modules together in order to move the robot honolomicaly.
 * The class contains several convenient methods for controlling the robot and retrieving information about his state.
 * <p>
 * The subsystem has the capability to work in both field oriented and robot oriented mode.
 */
public class SwerveDrive extends SubsystemBase {
    private static SwerveDrive FIELD_ORIENTED_INSTANCE = null;
    private static SwerveDrive ROBOT_ORIENTED_INSTANCE = null;
    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.SwerveDrive.SWERVE_POSITIONS);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    new Rotation2d(),
                    new Pose2d(),
                    kinematics,
                    VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                    VecBuilder.fill(Units.degreesToRadians(0.01)),
                    VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    private final ProfiledPIDController headingController = new ProfiledPIDController(
            Constants.SwerveDrive.HEADING_KP,
            Constants.SwerveDrive.HEADING_KI,
            Constants.SwerveDrive.HEADING_KD,
            Constants.SwerveDrive.HEADING_CONTROLLER_CONSTRAINTS
    );
    private final TimeDelayedBoolean rotationDelay = new TimeDelayedBoolean();
    private final boolean fieldOriented;
    private final Supplier<VisionEstimationData> visionPose;
    Timer timer = new Timer();
    double prevSpeed = 0;
    boolean bool = true;

    private SwerveDrive(boolean fieldOriented, Supplier<VisionEstimationData> visionPose) {
        this.fieldOriented = fieldOriented;
        this.visionPose = visionPose;
        modules[Constants.SwerveModule.frConfig.wheel()] = new SwerveModule(Constants.SwerveModule.frConfig);
        modules[Constants.SwerveModule.flConfig.wheel()] = new SwerveModule(Constants.SwerveModule.flConfig);
        modules[Constants.SwerveModule.rrConfig.wheel()] = new SwerveModule(Constants.SwerveModule.rrConfig);
        modules[Constants.SwerveModule.rlConfig.wheel()] = new SwerveModule(Constants.SwerveModule.rlConfig);

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.reset(0, 0);
        headingController.setTolerance(Constants.SwerveDrive.ALLOWABLE_HEADING_ERROR);
        timer.start();
        timer.reset();
        SmartDashboard.putNumber("max_vel_bruh", 0);
    }

    /**
     * @return the swerve in robot oriented mode.
     */
    public static SwerveDrive getRobotOrientedInstance(Supplier<VisionEstimationData> visionPose) {
        if (ROBOT_ORIENTED_INSTANCE == null) {
            ROBOT_ORIENTED_INSTANCE = new SwerveDrive(false, visionPose);
        }
        return ROBOT_ORIENTED_INSTANCE;
    }

    /**
     * @return the swerve in field oriented mode.
     */
    public static SwerveDrive getFieldOrientedInstance(Supplier<VisionEstimationData> visionPose) {
        if (FIELD_ORIENTED_INSTANCE == null) {
            FIELD_ORIENTED_INSTANCE = new SwerveDrive(true, visionPose);
        }
        return FIELD_ORIENTED_INSTANCE;
    }

    /**
     * Log the values of the inputs.
     *
     * @param speeds the speeds in each axis.
     */
    public static void logSpeeds(ChassisSpeeds speeds) {
        FireLog.log("current_angle", Robot.getAngle().getDegrees());
        FireLog.log("forward", speeds.vxMetersPerSecond);
        FireLog.log("strafe", speeds.vyMetersPerSecond);
        FireLog.log("rotation", speeds.omegaRadiansPerSecond);
    }

    /**
     * Gets the kinematics of the swerve.
     *
     * @return the kinematics of the swerve.
     */
    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    /**
     * Move the swerve in the specified direction, rotation and velocity.
     *
     * @param forward  the velocity on the X-axis. [m/s]
     * @param strafe   the velocity on the Y-axis. [m/s]
     * @param rotation the rotational velocity counter-clockwise positive. [rad/s]
     */
    public void holonomicDrive(double forward, double strafe, double rotation) {
//        if (rotation == 0 || rotationDelay.update(Math.abs(headingController.getGoal().position - Robot.getAngle()
//                        .getRadians()) < Constants.SwerveDrive.ALLOWABLE_HEADING_ERROR,
//                Constants.SwerveDrive.ROTATION_DELAY)) {
//            rotation = headingController.calculate(Robot.getAngle().getRadians());
//        } else {
//            headingController.setGoal(Robot.getAngle().getRadians());
//        }
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Robot.getAngle()) :
                new ChassisSpeeds(forward, strafe, rotation);
        setStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Move the swerve in the specified direction, rotation and velocity.
     *
     * @param forward  the velocity on the X-axis. [m/s]
     * @param strafe   the velocity on the Y-axis. [m/s]
     * @param rotation the rotational velocity counter-clockwise positive. [rad/s]
     */
    public void defaultHolonomicDrive(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Robot.getAngle()) :
                new ChassisSpeeds(forward, strafe, rotation);
        setStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Move the swerve in the specified direction, rotation and velocity.
     *
     * @param forward  the velocity on the X-axis. [m/s]
     * @param strafe   the velocity on the Y-axis. [m/s]
     * @param rotation the rotational velocity counter-clockwise positive. [rad/s]
     */
    public void errorRelativeHolonomicDrive(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Robot.getAngle()) :
                new ChassisSpeeds(forward, strafe, rotation);
        errorRelativeSetStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Set the states of the modules, but the velocities are relative to the angle error of the modules.
     *
     * @param states the states of the modules.
     */
    private void errorRelativeSetStates(SwerveModuleState[] states) {
        for (SwerveModule module : modules) {
            states[module.getWheel()] = SwerveModuleState.optimize(states[module.getWheel()], module.getAngle());
            double diff = states[module.getWheel()].angle.minus(module.getAngle()).getRadians();
            module.setAngle(states[module.getWheel()].angle);
            module.setVelocity(states[module.getWheel()].speedMetersPerSecond * Math.cos(diff));
        }
    }

    /**
     * Check whether all modules have reached their desired angles.
     *
     * @param forward  the velocity on the X-axis. [m/s]
     * @param strafe   the velocity on the Y-axis. [m/s]
     * @param rotation the rotational velocity counter-clockwise positive. [rad/s]
     */
    public boolean haveModulesReachedAngles(double forward, double strafe, double rotation) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                forward, strafe, rotation, Robot.getAngle());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        for (SwerveModule module : modules) {
            states[module.getWheel()] = SwerveModuleState.optimize(states[module.getWheel()], module.getAngle());
            if (!(Math.abs(states[module.getWheel()].angle.minus(module.getAngle()).getDegrees()) < 3)) { // TODO: Remove the magic number
                return false;
            }
        }
        return true;
    }

    /**
     * Rotates the robot to a desired angle.
     *
     * @param desiredAngle the desired angle of the robot.
     */
    public double getHeadingOutput(Rotation2d desiredAngle) {
        double lastGoal = headingController.getGoal().position;
        headingController.setGoal(desiredAngle.getRadians());
        double output = headingController.calculate(Robot.getAngle().getRadians());

        headingController.setGoal(lastGoal);
        return output;
    }

    /**
     * Gets the states of every module.
     *
     * @return the states of every module.
     */
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] swerveModuleState = new SwerveModuleState[modules.length];
        for (SwerveModule module : modules) {
            swerveModuleState[module.getWheel()] = module.getState();
        }
        return swerveModuleState;
    }

    /**
     * Sets the state of the modules.
     *
     * @param states the states of the modules.
     */
    public void setStates(SwerveModuleState[] states) {
        for (SwerveModule module : modules) {
            states[module.getWheel()] = SwerveModuleState.optimize(states[module.getWheel()], module.getAngle());
            module.setState(states[module.getWheel()]);
        }
    }

    public void desaturatedSetStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Autonomous.MAX_VEL);
        for (SwerveModule module : modules) {
            states[module.getWheel()] = SwerveModuleState.optimize(states[module.getWheel()], module.getAngle());
            module.setState(states[module.getWheel()]);
        }
    }

    /**
     * Gets the chassis speeds of the entire robot.
     *
     * @return the speed of the robot in each axis.
     */
    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getStates());
        if (!fieldOriented) {
            return chassisSpeeds;
        }
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                chassisSpeeds.omegaRadiansPerSecond,
                Robot.getAngle()
        );
    }

    /**
     * Gets a specific module, shouldn't be used for regular cases.
     *
     * @param index the index of the module.
     * @return the corresponding module.
     */
    public SwerveModule getModule(int index) {
        return modules[index];
    }

    /**
     * Gets the pose of the robot.
     *
     * @return the pose of the robot.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Resets the odometry.
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d(), new Rotation2d());
    }

    /**
     * Resets the odometry to a specified position.
     *
     * @param pose the current pose.
     */
    public void resetOdometry(Pose2d pose, Rotation2d angle) {
        odometry.resetPosition(new Pose2d(pose.getTranslation(), angle), angle);
    }

    /**
     * Resets the heading controller target angle.
     */
    public void resetHeadingController() {
        headingController.reset(0, getChassisSpeeds().omegaRadiansPerSecond);
    }

    /**
     * Set the power of the angle motors for each module to a specified percent power for testing purposes.
     *
     * @param power percent power to give to the angel motors. [%]
     */
    public void setPower(double power) {
        for (var module : modules) {
            module.setPower(power);
        }
    }

    /**
     * Sets the state of the modules without optimizing them.
     * USE ONLY FOR TESTING & TUNING!
     *
     * @param states the states of the modules.
     */
    public void noOptimizeSetStates(SwerveModuleState[] states) {
        for (SwerveModule module : modules) {
            module.setState(states[module.getWheel()]);
        }
    }

    /**
     * Terminates the modules from moving.
     */
    public void terminate() {
        for (SwerveModule module : modules) {
            module.stopDriveMotor();
            module.stopAngleMotor();
        }
    }

    @Override
    public void periodic() {
        odometry.updateWithTime(
                Timer.getFPGATimestamp(),
                Robot.getAngle(),
                getStates()
        );

        poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Robot.getAngle(),
                getStates()
        );

        var visionData = visionPose.get();
        if (visionData.hasTarget()) {
            poseEstimator.addVisionMeasurement(visionData.estimatedPose(), visionData.time());
        }

        String outputPosition = poseEstimator.getEstimatedPosition().getX() + ", " + poseEstimator.getEstimatedPosition().getY();
        SmartDashboard.putString("robot_position", outputPosition);

//        String outputPosition = getPose().getX() + ", " + getPose().getY();
//        SmartDashboard.putString("robot_position", outputPosition);
        String outputRotation = String.valueOf(Robot.getAngle().getDegrees());
        SmartDashboard.putString("robot_rotation", outputRotation);


    }
}

