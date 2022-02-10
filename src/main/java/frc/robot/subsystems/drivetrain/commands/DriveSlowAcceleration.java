package frc.robot.subsystems.drivetrain.commands;

import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

public class DriveSlowAcceleration extends HolonomicDrive {
    private double current = 0;

    public DriveSlowAcceleration(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        super(swerveDrive, forwardSupplier, strafeSupplier, rotationSupplier);
    }

    @Override
    public void execute() {
        var speeds = calculateVelocities();
        double magnitude = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        current += magnitude / 10;
        if (current > magnitude) current = magnitude;
        double alpha = Math.atan2(speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);

        double forward = current * Math.sin(alpha);
        double strafe = current * Math.cos(alpha);
        double rotation = speeds.omegaRadiansPerSecond;

        if (magnitude == 0 && rotation == 0)
            swerveDrive.terminate();
        else {
            swerveDrive.holonomicDrive(forward, strafe, rotation);
        }
        SwerveDrive.logSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
