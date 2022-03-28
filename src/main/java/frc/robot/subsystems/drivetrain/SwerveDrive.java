package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import webapp.FireLog;

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
    private final SwerveDriveOdometry odometryForDistance = new SwerveDriveOdometry(kinematics, new Rotation2d());

    private final ProfiledPIDController headingController = new ProfiledPIDController(
            Constants.SwerveDrive.HEADING_KP,
            Constants.SwerveDrive.HEADING_KI,
            Constants.SwerveDrive.HEADING_KD,
            Constants.SwerveDrive.HEADING_CONTROLLER_CONSTRAINTS
    );
    private final boolean fieldOriented;

    private final DoubleLogEntry xVelocity;
    private final DoubleLogEntry yVelocity;
    private final DoubleLogEntry rotationVelocity;

    private SwerveDrive(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
        modules[Constants.SwerveModule.frConfig.wheel()] = new SwerveModule(Constants.SwerveModule.frConfig);
        modules[Constants.SwerveModule.flConfig.wheel()] = new SwerveModule(Constants.SwerveModule.flConfig);
        modules[Constants.SwerveModule.rrConfig.wheel()] = new SwerveModule(Constants.SwerveModule.rrConfig);
        modules[Constants.SwerveModule.rlConfig.wheel()] = new SwerveModule(Constants.SwerveModule.rlConfig);

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.reset(0, 0);
        headingController.setTolerance(Constants.SwerveDrive.ALLOWABLE_HEADING_ERROR);

        DataLog log = DataLogManager.getLog();
        xVelocity = new DoubleLogEntry(log, "/swerveDrive/x-velocity");
        yVelocity = new DoubleLogEntry(log, "/swerveDrive/y-velocity");
        rotationVelocity = new DoubleLogEntry(log, "/swerveDrive/rotational-velocity");

    }

    /**
     * @return the swerve in robot oriented mode.
     */
    public static SwerveDrive getRobotOrientedInstance() {
        if (ROBOT_ORIENTED_INSTANCE == null) {
            ROBOT_ORIENTED_INSTANCE = new SwerveDrive(false);
        }
        return ROBOT_ORIENTED_INSTANCE;
    }

    /**
     * @return the swerve in field oriented mode.
     */
    public static SwerveDrive getFieldOrientedInstance() {
        if (FIELD_ORIENTED_INSTANCE == null) {
            FIELD_ORIENTED_INSTANCE = new SwerveDrive(true);
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
            if (!(Math.abs(states[module.getWheel()].angle.minus(module.getAngle())
                    .getDegrees()) < 7)) { // TODO: Remove the magic number
                return false;
            }
        }
        return true;
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

    public Pose2d getDistanceOdometryPose() {
        return odometryForDistance.getPoseMeters();
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


    public void resetDistanceOdometry(Pose2d pose, Rotation2d angle) {
        odometryForDistance.resetPosition(new Pose2d(pose.getTranslation(), angle), angle);
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

//        lock();
    }

    public void lock() {
        modules[0].setAngle(Rotation2d.fromDegrees(45));
        modules[1].setAngle(Rotation2d.fromDegrees(-45));
        modules[3].setAngle(Rotation2d.fromDegrees(45));
        modules[2].setAngle(Rotation2d.fromDegrees(-45));
    }

    /**
     * Get the distance of the robot from the hub using odometry.
     *
     * @return distance from hub. [m]
     */
    public double getOdometryDistance() {
        return Constants.Vision.HUB_POSE.getTranslation().minus(getDistanceOdometryPose().getTranslation()).getNorm();
    }

    @Override
    public void periodic() {
        odometry.updateWithTime(
                Timer.getFPGATimestamp(),
                Robot.getAngle(),
                getStates()
        );

        odometryForDistance.updateWithTime(
                Timer.getFPGATimestamp(),
                Robot.getAngle(),
                getStates()
        );

        String outputRotation = String.valueOf(Robot.getAngle().getDegrees());
        SmartDashboard.putString("robot_rotation", outputRotation);

        var speeds = getChassisSpeeds();
        xVelocity.append(speeds.vxMetersPerSecond);
        yVelocity.append(speeds.vyMetersPerSecond);
        rotationVelocity.append(speeds.omegaRadiansPerSecond);
    }

    public void setPowerVelocity() {
        for (var module : modules)  {
            module.setVelocity(100);
        }
    }
}

