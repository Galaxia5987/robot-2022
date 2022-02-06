package frc.robot.subsystems.drivetrain.commands.tuning;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.WebConstant;
import webapp.FireLog;

/**
 * The command is responsible for tuning the velocities of the drive motor.
 * The command rotates the velocities the motor at a specified rate and log it into Fielog.
 */
public class DriveForward extends CommandBase {

    private final SwerveDrive swerveDrive;
    private final WebConstant target = WebConstant.of("Swerve", "targetVelocity", 0);

    public DriveForward(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setState(new SwerveModuleState(target.get(), new Rotation2d(0)));
        }
        FireLog.log("target velocity", target.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}