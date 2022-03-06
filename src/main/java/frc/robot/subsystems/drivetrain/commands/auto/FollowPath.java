package frc.robot.subsystems.drivetrain.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class FollowPath extends CommandBase {
    private final Timer m_timer = new Timer();
    private final PathPlannerTrajectory m_trajectory;
    private final Supplier<Pose2d> m_pose;
    private final SwerveDriveKinematics m_kinematics;
    private final HolonomicDriveController m_controller;
    private final Consumer<SwerveModuleState[]> m_outputModuleStates;
    private final PIDController thetaController = new PIDController(Constants.Autonomous.KP_THETA_CONTROLLER, 0, 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
    }};

    /**
     * Constructs a new PPSwerveControllerCommand that when executed will follow the
     * provided
     * trajectory. This command will not return output voltages but rather raw
     * module states from the
     * position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion
     * of the path-
     * this is left to the user, since it is not appropriate for paths with
     * nonstationary endstates.
     *
     * @param trajectory         The trajectory to follow.
     * @param pose               A function that supplies the robot pose - use one
     *                           of the odometry classes to
     *                           provide this.
     * @param kinematics         The kinematics for the robot drivetrain.
     * @param xController        The Trajectory Tracker PID controller for the
     *                           robot's x position.
     * @param yController        The Trajectory Tracker PID controller for the
     *                           robot's y position.
     * @param outputModuleStates The raw output module states from the position
     *                           controllers.
     * @param requirements       The subsystems to require.
     */
    @SuppressWarnings("ParameterName")
    public FollowPath(
            PathPlannerTrajectory trajectory,
            Supplier<Pose2d> pose,
            SwerveDriveKinematics kinematics,
            PIDController xController,
            PIDController yController,
            Consumer<SwerveModuleState[]> outputModuleStates,
            Subsystem... requirements) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_kinematics = kinematics;

        m_controller = new HolonomicDriveController(
                xController,
                yController,
                new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0)));

        m_outputModuleStates = outputModuleStates;

        addRequirements(requirements);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    @SuppressWarnings("LocalVariableName")
    public void execute() {
        double curTime = m_timer.get();
        var desiredState = (PathPlannerTrajectory.PathPlannerState) m_trajectory.sample(curTime);

        var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);
        var targetModuleStates = m_kinematics.toSwerveModuleStates(
                new ChassisSpeeds(targetChassisSpeeds.vxMetersPerSecond, targetChassisSpeeds.vyMetersPerSecond, thetaController.calculate(
                        Robot.getAngle().getRadians(), desiredState.holonomicRotation.getRadians())));

        m_outputModuleStates.accept(targetModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}