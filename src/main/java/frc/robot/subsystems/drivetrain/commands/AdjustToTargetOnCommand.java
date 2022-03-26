package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class AdjustToTargetOnCommand extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier yawSupplier;
    private final PIDController adjustController = new PIDController(Constants.SwerveDrive.ADJUST_CONTROLLER_KP.get(), 0, 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
        setTolerance(Constants.SwerveDrive.ADJUST_CONTROLLER_TOLERANCE);
    }};
    private final BooleanSupplier hasTarget;
    private Rotation2d target;


    public AdjustToTargetOnCommand(SwerveDrive swerveDrive, DoubleSupplier yawSupplier, BooleanSupplier hasTarget) {
        this.swerveDrive = swerveDrive;
        this.yawSupplier = yawSupplier;
        this.hasTarget = hasTarget;
        target = Robot.getAngle();
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        if (!hasTarget.getAsBoolean()) {
            var robotPose = swerveDrive.getPose().getTranslation();
            var hubPose = Constants.Vision.HUB_POSE.getTranslation();
            var poseRelativeToTarget = hubPose.minus(robotPose);
            var value = new Rotation2d(
                    Math.atan2(
                            poseRelativeToTarget.getY(),
                            poseRelativeToTarget.getX()
                    ));
            target = value;
        } else {
            target = Robot.getAngle().minus(Rotation2d.fromDegrees(yawSupplier.getAsDouble()));
        }
    }

    @Override
    public void execute() {
        double rotation = adjustController.calculate(Robot.getAngle().getRadians(), target.getRadians());
        swerveDrive.holonomicDrive(0, 0, rotation);
    }


    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
