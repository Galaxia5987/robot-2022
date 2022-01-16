package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

public class PrepareShooter extends CommandBase {
    private final Shooter shooter;
    private final double velocitySetpoint;

    public PrepareShooter(Shooter shooter, double velocitySetpoint) {
        this.shooter = shooter;
        this.velocitySetpoint = velocitySetpoint;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocitySetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
