package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class FeedToShooter extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier setpointVelocity;

    public FeedToShooter(Shooter shooter, DoubleSupplier setpointVelocity) {
        this.shooter = shooter;
        this.setpointVelocity = setpointVelocity;
    }

    @Override
    public void execute() {
        if (shooter.getVelocity() == setpointVelocity.getAsDouble()) {
            // Feed to the shooter.
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop feeding to the shooter.
    }
}
