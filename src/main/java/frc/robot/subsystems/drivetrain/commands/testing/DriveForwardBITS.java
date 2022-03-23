package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class DriveForwardBITS extends CommandBase {
    private final SwerveDrive swerveDrive;

    public DriveForwardBITS(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.setPowerVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}
