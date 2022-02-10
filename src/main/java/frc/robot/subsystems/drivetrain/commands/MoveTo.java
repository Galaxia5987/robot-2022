package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.List;
import java.util.function.Supplier;

/**
 * The purpose of this command is to move to the position of the cargo, but can be used for tracking other objects.
 */
public class MoveTo extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final TrajectoryConfig config;
    private final Supplier<Pose2d> targetPose;
    private final HolonomicDriveController controller;
    private final Timer timer = new Timer();
    private Pose2d lastPose;
    private Trajectory lastTrajectory;

    public MoveTo(SwerveDrive swerveDrive, TrajectoryConfig config, HolonomicDriveController controller, Supplier<Pose2d> targetPose) {
        this.swerveDrive = swerveDrive;
        this.config = config;
        this.controller = controller;
        this.targetPose = targetPose;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        lastPose = targetPose.get();
        lastTrajectory = TrajectoryGenerator.generateTrajectory(List.of(swerveDrive.getPose(), lastPose), config);
        timer.start();
    }

    @Override
    public void execute() {
        var currentTargetPose = targetPose.get();
        if (!currentTargetPose.equals(lastPose)) {
            lastTrajectory = TrajectoryGenerator.generateTrajectory(List.of(swerveDrive.getPose(), currentTargetPose), config);
            lastPose = currentTargetPose;
            timer.reset();
        }
        var state = lastTrajectory.sample(timer.get());
        var targetChassisSpeeds =
                controller.calculate(swerveDrive.getPose(), state, currentTargetPose.getRotation());

        swerveDrive.holonomicDrive(targetChassisSpeeds.vxMetersPerSecond,
                targetChassisSpeeds.vxMetersPerSecond,
                targetChassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
