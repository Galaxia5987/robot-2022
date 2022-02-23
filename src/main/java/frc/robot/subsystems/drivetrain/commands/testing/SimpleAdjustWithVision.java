package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SimpleAdjustWithVision extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier condition;
    private final DoubleSupplier yawSupplier;
    private final PIDController adjustController = new PIDController(Constants.SwerveDrive.ADJUST_CONTROLLER_KP, Constants.SwerveDrive.ADJUST_CONTROLLER_KI.get(), 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
        setTolerance(Constants.SwerveDrive.ADJUST_CONTROLLER_TOLERANCE);
    }};
    private final DoubleSupplier distanceSupplier;
    private Rotation2d target;
    private boolean last = false;


    public SimpleAdjustWithVision(SwerveDrive swerveDrive, DoubleSupplier rotationSupplier, BooleanSupplier condition, DoubleSupplier yawSupplier, DoubleSupplier distanceSupplier) {
        this.swerveDrive = swerveDrive;
        this.rotationSupplier = rotationSupplier;
        this.condition = condition;
        this.yawSupplier = yawSupplier;
        this.distanceSupplier = distanceSupplier;
        target = Robot.getAngle();
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        System.out.println(condition.getAsBoolean() + " " + yawSupplier.getAsDouble() + " " + distanceSupplier.getAsDouble());
        double rotation = Utils.deadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        if (rotation == 0 && !condition.getAsBoolean()) {
            swerveDrive.terminate();
            last = false;
        } else {
            if (condition.getAsBoolean()) {
                if (!last) {
                    Rotation2d offset = new Rotation2d(Math.atan2(Math.toRadians(-Math.signum(yawSupplier.getAsDouble()) * Constants.Shooter.CARGO_OFFSET), distanceSupplier.getAsDouble()));
                    target = Robot.getAngle().minus(Rotation2d.fromDegrees(yawSupplier.getAsDouble()).plus(offset));
                    last = true;
                }
                rotation = adjustController.calculate(Robot.getAngle().getRadians(), target.getRadians());
            } else {
                last = false;
            }
            swerveDrive.holonomicDrive(0, 0, rotation);

        }
    }


    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
