package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utils.TimeDelayedBoolean;


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
    private final ProfiledPIDController headingController = new ProfiledPIDController(
            Constants.SwerveDrive.THETA_KP,
            Constants.SwerveDrive.THETA_KI,
            Constants.SwerveDrive.THETA_KD,
            new TrapezoidProfile.Constraints(Constants.SwerveDrive.ROTATION_MULTIPLIER, Constants.SwerveDrive.ROTATION_MULTIPLIER / 2)
    );
    private final TimeDelayedBoolean rotationDelay = new TimeDelayedBoolean();
    private final boolean fieldOriented;

    public SwerveDrive(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
        modules[Constants.SwerveModule.frConfig.wheel()] = new SwerveModule(Constants.SwerveModule.frConfig);
        modules[Constants.SwerveModule.flConfig.wheel()] = new SwerveModule(Constants.SwerveModule.flConfig);
        modules[Constants.SwerveModule.rrConfig.wheel()] = new SwerveModule(Constants.SwerveModule.rrConfig);
        modules[Constants.SwerveModule.rlConfig.wheel()] = new SwerveModule(Constants.SwerveModule.rlConfig);

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.reset(0, 0);
        headingController.setTolerance(Constants.SwerveDrive.ALLOWABLE_THETA_ERROR);
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
     * @param forward  the velocity on the Y-axis. [m/s]
     * @param strafe   the velocity on the X-axis. [m/s]
     * @param rotation rhe rotational velocity. [rad/s]
     */
    public void holonomicDrive(double forward, double strafe, double rotation) {
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, Robot.getAngle()) :
                new ChassisSpeeds(forward, strafe, rotation);
        setStates(kinematics.toSwerveModuleStates(speeds));

    }

    /**
     * Rotates the robot to a desired angle.
     *
     * @param desiredAngle the desired angle of the robot.
     */
    public void holonomicDriveKeepSetpoint(double forward, double strafe, Rotation2d desiredAngle) {
        headingController.setGoal(desiredAngle.getRadians());
        double output = headingController.calculate(Robot.getAngle().getRadians());

        setStates(kinematics.toSwerveModuleStates(new ChassisSpeeds(forward, strafe, output)));
    }

    /**
     * Gets te states of every module.
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
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.42 * Constants.SwerveDrive.VELOCITY_MULTIPLIER);
        for (SwerveModule module : modules) {
            SwerveModuleState state = states[module.getWheel()];
            state = SwerveModuleState.optimize(state, module.getAngle());
            module.setState(state);
        }
    }

    /**
     * Gets the chassis speeds of the entire robot.
     *
     * @return the speed of the robot in each axis.
     */
    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getStates());
        if (fieldOriented) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeeds.vxMetersPerSecond,
                    chassisSpeeds.vyMetersPerSecond,
                    chassisSpeeds.omegaRadiansPerSecond,
                    Robot.getAngle()
            );
        }
        return chassisSpeeds;
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
     * @param pose     the current pose.
     * @param rotation the holonomic rotation.
     */
    public void resetOdometry(Pose2d pose, Rotation2d rotation) {
        odometry.resetPosition(new Pose2d(pose.getTranslation(), rotation), rotation);
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
        for (int i = 0; i < 4; i++) {
            getModule(i).setPower(power);
        }
    }

    /**
     * Test the slippage of the angle motor encoders by setting all the modules to their zero positions and running this function.
     */
    public void testEncoderSlippage() {
        for (int i = 0; i < 4; i++) {
            int error = (int) (Math.abs(getModule(i).getSelectedSensorPosition() - Constants.SwerveModule.ZERO_POSITIONS[i]) % Constants.SwerveDrive.TICKS_PER_ROTATION_ANGLE_MOTOR);
            if (error < 5) {
                System.out.println("WHEEL: " + i + "is PERFECT! Error is: " + error);
                SmartDashboard.putString("EncoderSlippageResult [WHEEL " + i + "]", "WHEEL: " + i + "is PERFECT! Error is: " + error);
            } else if (error < 20) {
                System.out.println("WHEEL: " + i + "is GOOD. Error is: " + error);
                SmartDashboard.putString("EncoderSlippageResult [WHEEL " + i + "]", "WHEEL: " + i + "is Good. Error is: " + error);
            } else {
                System.out.println("WHEEL: " + i + "is BAD! Error is: " + error);
                SmartDashboard.putString("EncoderSlippageResult [WHEEL " + i + "]", "WHEEL: " + i + "is BAD! Error is: " + error);
            }
        }
    }


    public void printEncoderReadings() {
        System.out.println(getModule(0).getSelectedSensorPosition() + ", " + getModule(1).getSelectedSensorPosition() + ", " + getModule(2).getSelectedSensorPosition() + ", " + getModule(3).getSelectedSensorPosition());
    }

    /**
     * Set all the modules to their desired positions without much velocity for smoothing purposes.
     *
     * @param vx  velocity for the x-axis. [m/s]
     * @param vy  velocity for the x-axis. [m/s]
     * @param rot rotational velocity counter-clockwise positive. [rad/s]
     */
    public void noSpeedHolonomicDrive(double vx, double vy, double rot) {
        ChassisSpeeds speeds = fieldOriented ?
                ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, rot, Robot.getAngle()) :
                new ChassisSpeeds(vx, vy, rot);
        noSpeedSetStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * Set all the modules to their desired positions without much velocity for smoothing purposes.
     *
     * @param states desired angles and velocities for each module.
     */
    public void noSpeedSetStates(SwerveModuleState[] states) {
        for (int i = 0; i < 4; i++) {
            double diff = Math.abs(states[i].angle.minus(getModule(i).getAngle()).getRadians());
            states[i] = SwerveModuleState.optimize(states[i], getModule(i).getAngle());
            getModule(i).setAngle(states[i].angle);
            getModule(i).setVelocity(states[i].speedMetersPerSecond * Math.abs(Math.cos(diff)));//comment this out
        }
    }


    /**
     * Check whether all modules have reached their desired angles.
     *
     * @param vx  velocity for the x-axis. [m/s]
     * @param vy  velocity for the x-axis. [m/s]
     * @param rot rotational velocity counter-clockwise positive. [rad/s]
     * @return whether all modules have reached their desired angles.
     */
    public boolean haveReachedAngles(double vx, double vy, double rot) {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx, vy, rot, Robot.getAngle());
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        for (int i = 0; i < 4; i++)
            states[i] = SwerveModuleState.optimize(states[i], getModule(i).getAngle());

        for (int i = 0; i < 4; i++) {
            if (!(Math.abs(states[i].angle.minus(getModule(i).getAngle()).getDegrees()) < 10)) {
                return false;
            }
        }
        return true;
    }

    /**
     * Sets the state of the modules without optimizing them.
     *
     * @param states the states of the modules.
     */
    public void noOptimizeSetStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.42 * Constants.SwerveDrive.VELOCITY_MULTIPLIER);
        for (SwerveModule module : modules) {
            SwerveModuleState state = states[module.getWheel()];
            module.setState(state);
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

        System.out.println("To reset zero positions, 1. disable the robot, 2. rotate the wheel so that the black part of the wheel is facing the drawn y arrow, 3. under the Constants class -> SwerveModule -> ZERO_POSITIONS -> Change the values inside the brackets to:");
        printEncoderReadings();
/*
        headingController.setP(Constants.SwerveDrive.THETA_KP.get());
        headingController.setI(Constants.SwerveDrive.THETA_KI.get());
        headingController.setD(Constants.SwerveDrive.THETA_KD.get());
*/
    }
}
