package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AssistedDrive extends HolonomicDrive {
    PIDController pidController = new PIDController(0.8, 0, 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
        setTolerance(5);
    }};
    private boolean keepSetpoint = false;
    private Rotation2d setpoint = new Rotation2d();

    public AssistedDrive(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        super(swerveDrive, forwardSupplier, strafeSupplier, rotationSupplier);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        double forward = speeds.vyMetersPerSecond, strafe = speeds.vxMetersPerSecond, rotation = speeds.omegaRadiansPerSecond;

        if (forward == 0 && strafe == 0 && rotation == 0) {
            swerveDrive.terminate();
            keepSetpoint = false;
        } else {
            if (rotation != 0) {
                keepSetpoint = false;
            } else if (!keepSetpoint) {
                setpoint = Robot.navx.getRotation2d();
                keepSetpoint = true;
            }
            if (keepSetpoint) {
                swerveDrive.holonomicDrive(forward, strafe, pidController.calculate(Robot.getAngle().getRadians(), setpoint.getRadians()));
            } else {
                swerveDrive.holonomicDrive(forward, strafe, rotation);
            }
        }
        log(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
