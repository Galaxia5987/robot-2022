package frc.robot.commandgroups;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class VisionAdjust extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final PIDController adjustController = new PIDController(Constants.SwerveDrive.ADJUST_CONTROLLER_KP.get(), 0, 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
        setTolerance(Constants.SwerveDrive.ADJUST_CONTROLLER_TOLERANCE);
    }};
    private Rotation2d target;


    public VisionAdjust(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        target = Robot.getAngle();
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        target = Robot.getAngle().minus(Rotation2d.fromDegrees(RobotContainer.Suppliers.yawSupplier.getAsDouble()));
    }

    @Override
    public void execute() {
        double rotation = adjustController.calculate(Robot.getAngle().getRadians(), target.getRadians());
        swerveDrive.holonomicDrive(0, 0, rotation);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Robot.getAngle().minus(target).getDegrees()) < Constants.SwerveDrive.VISION_ADJUST_FINISHED_CONDITION;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
