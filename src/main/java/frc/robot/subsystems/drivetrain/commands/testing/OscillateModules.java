package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class OscillateModules extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();

    public OscillateModules(SwerveDrive swerve) {
        this.swerve = swerve;
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(0.1)) {
            for (int i = 0; i < 4; i++) {
                swerve.getModule(i).setAngle(Rotation2d.fromDegrees(360 * Math.random()));
            }
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        for (int i = 0; i < 4; i++) {
            swerve.getModule(i).setAngle(Rotation2d.fromDegrees(0));
        }
    }
}
