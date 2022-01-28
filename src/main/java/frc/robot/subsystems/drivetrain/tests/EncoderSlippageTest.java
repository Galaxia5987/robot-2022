package frc.robot.subsystems.drivetrain.tests;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.SwerveDrive;


public class EncoderSlippageTest extends CommandBase {
    private final SwerveDrive swerve;
    private final Timer timer = new Timer();
    private final Timer secondary = new Timer();
    double power = 1;
    double dt = 0.25;

    public EncoderSlippageTest(SwerveDrive swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        super.initialize();
        timer.reset();
        timer.start();
        secondary.reset();
        secondary.start();
    }

    @Override
    public void execute() {
        if (secondary.hasElapsed(dt)) {
            power = Math.random() * 2 - 1;
            power = Math.signum(power) * MathUtil.clamp(Math.abs(power), 0.7, 1);
            dt = MathUtil.clamp(Math.random() / 2, 0.1, 0.3);
            secondary.reset();
        }
        swerve.setPower(power);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        timer.stop();
        secondary.stop();
        swerve.terminate();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(10);
    }
}
