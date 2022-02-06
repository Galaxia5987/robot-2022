package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Rotates the entire chassis to a specified angle.
 */
public class TurnToAngle extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final Supplier<Rotation2d> targetAngle;

    /**
     * Initialize rotate to angle command.
     *
     * @param swerveDrive the SwerveDrive subsystem
     * @param targetAngle the target angle. [rad]
     */
    public TurnToAngle(SwerveDrive swerveDrive, DoubleSupplier targetAngle) {
        this.swerveDrive = swerveDrive;
        this.targetAngle = () -> new Rotation2d(targetAngle.getAsDouble());
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        double output = swerveDrive.getHeadingOutput(targetAngle.get());

        swerveDrive.holonomicDrive(0, 0, output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(Robot.getAngle()
                .minus(targetAngle.get())
                .getRadians()) < Constants.SwerveDrive.ALLOWABLE_THETA_ERROR;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}