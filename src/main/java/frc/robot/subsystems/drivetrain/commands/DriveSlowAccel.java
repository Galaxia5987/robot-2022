package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import static frc.robot.Constants.SwerveDrive.ROTATION_MULTIPLIER;
import static frc.robot.Constants.SwerveDrive.VELOCITY_MULTIPLIER;

public class DriveSlowAccel extends CommandBase {
    private final SwerveDrive swerve;
    private double current = 0;

    public DriveSlowAccel(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        double forward = -RobotContainer.xbox.getLeftY();
        double strafe = -RobotContainer.xbox.getLeftX();
        double magnitude = Math.hypot(forward, strafe);
        double alpha = Math.atan2(strafe, forward);
        if (Math.abs(magnitude) < 0.1)
            magnitude = 0;
        if (magnitude == 0) current = 0;
        magnitude *= VELOCITY_MULTIPLIER;
        current += magnitude / 25;
        if (current > magnitude) current = magnitude;
        forward = Math.cos(alpha) * current;
        strafe = Math.sin(alpha) * current;
        double rotation = -RobotContainer.xbox.getRightX();
        if (Math.abs(rotation) < 0.1) rotation = 0;
        rotation *= ROTATION_MULTIPLIER;

        if (magnitude == 0 && rotation == 0)
            swerve.terminate();
        else {
            swerve.holonomicDrive(forward, strafe, rotation);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.terminate();
    }
}
