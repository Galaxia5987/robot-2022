package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class PrepareShooter extends CommandBase {
    private final Shooter shooter = Shooter.getInstance();
    private final Timer timer = new Timer();
    private final double requiredVelocity;
    private double currentTime;
    private double lastTime = 0;

    public PrepareShooter(double requiredVelocity) {
        this.requiredVelocity = requiredVelocity;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        currentTime = timer.get();
        shooter.setVelocity(requiredVelocity, currentTime - lastTime);
        lastTime = currentTime;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
