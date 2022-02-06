package frc.robot.subsystems.drivetrain.commands.tuning;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.valuetuner.WebConstant;
import webapp.FireLog;

/**
 * The Rotate command rotate the modules to a specified angle (in degrees) so it will
 * be easier to tune the PID constants of the angle motor.
 */
public class Rotate extends CommandBase {
    private final SwerveDrive swerveDrive;
    private final WebConstant targetAngle = WebConstant.of("Swerve", "targetAngle", 0);

    public Rotate(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        for (int i = 0; i < 4; i++) {
            swerveDrive.getModule(i).setAngle(Rotation2d.fromDegrees(targetAngle.get()));
        }

        FireLog.log("angle-setpoint", targetAngle.get());
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}