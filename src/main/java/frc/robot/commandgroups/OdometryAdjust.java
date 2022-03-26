package frc.robot.commandgroups;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class OdometryAdjust extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final PIDController adjustController = new PIDController(8, 0, 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
        setTolerance(Constants.SwerveDrive.ADJUST_CONTROLLER_TOLERANCE);
    }};
    private Rotation2d target;


    public OdometryAdjust(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        target = new Rotation2d();
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        var robotPose = swerveDrive.getPose().getTranslation();
        var hubPose = Constants.Vision.HUB_POSE.getTranslation();
        var poseRelativeToTarget = hubPose.minus(robotPose);
        var value = new Rotation2d(
                Math.atan2(
                        poseRelativeToTarget.getY(),
                        poseRelativeToTarget.getX()
                ));
        target = value;
    }

    @Override
    public void execute() {
        double rotation = adjustController.calculate(Robot.getAngle().getRadians(), target.getRadians());
        swerveDrive.holonomicDrive(0, 0, rotation);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Robot.getAngle().minus(target).getDegrees()) < Constants.SwerveDrive.ODOMETRY_ADJUST_FINISHED_CONDITION;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
