package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

/**
 * The command is responsible for moving the robot holonomically.
 * The commands retrieve the velocities in every axis (including rotational) and moves the robot accordingly.
 */
public class HolonomicDrive extends CommandBase {
    protected final SwerveDrive swerveDrive;
    protected final DoubleSupplier forwardSupplier;
    protected final DoubleSupplier strafeSupplier;
    protected final DoubleSupplier rotationSupplier;

    public HolonomicDrive(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        this.swerveDrive = swerveDrive;
        this.forwardSupplier = forwardSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        swerveDrive.holonomicDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        SwerveDrive.logSpeeds(speeds);
    }

    /**
     * Calculates the speeds from the suppliers after filtering.
     */
    protected ChassisSpeeds calculateVelocities() {
        // get the values
        double forward = forwardSupplier.getAsDouble(); // vx
        double strafe = strafeSupplier.getAsDouble(); // vy
        double rotation = Utils.rotationalDeadband(rotationSupplier.getAsDouble(), Constants.SwerveDrive.JOYSTICK_THRESHOLD) * Constants.SwerveDrive.ROTATION_MULTIPLIER;

        // recalculate - update based on the angle and the magnitude
        double alpha = Math.atan2(strafe, forward); // direction of movement
        double magnitude = Utils.deadband(Math.hypot(forward, strafe) * Constants.SwerveDrive.VELOCITY_MULTIPLIER, Constants.SwerveDrive.JOYSTICK_THRESHOLD);
        forward = Math.cos(alpha) * magnitude;
        strafe = Math.sin(alpha) * magnitude;
        return new ChassisSpeeds(forward, strafe, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}