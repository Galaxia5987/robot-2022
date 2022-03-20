package frc.robot.subsystems.drivetrain.commands.testing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;

public class TurnAllModuleMotors extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();

    public TurnAllModuleMotors(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        for (int i = 0; i < 4; i++) {
            swerve.getModule(i).setPower(0.5);
            swerve.getModule(i).setAngle(Rotation2d.fromDegrees(1080 * timer.get()));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.terminate();
    }
}
