package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.DoubleSupplier;

public class WaitBeforeAccel extends HolonomicDrive {
    private boolean wait = false;

    public WaitBeforeAccel(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier) {
        super(swerveDrive, forwardSupplier, strafeSupplier, rotationSupplier);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        double forward = speeds.vxMetersPerSecond, strafe = speeds.vyMetersPerSecond, rotation = speeds.omegaRadiansPerSecond;

        if (forward == 0 && strafe == 0 && rotation == 0) {
            swerveDrive.terminate();
            wait = true;
        } else {
            if (swerveDrive.haveReached(forward, strafe, rotation))
                wait = false;
            if (wait) {
                swerveDrive.noSpeedSetChassisSpeedsStateSpace(forward, strafe, rotation);
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
