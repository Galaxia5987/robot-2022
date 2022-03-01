package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class WarmUpShooter extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier distanceFromTarget;

    public WarmUpShooter(Shooter shooter, DoubleSupplier distanceFromTarget) {
        this.shooter = shooter;
        this.distanceFromTarget = distanceFromTarget;
    }

    @Override
    public void execute() {
        double velocity = Shoot.getSetpointVelocity(
                distanceFromTarget.getAsDouble(), distanceFromTarget.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD);
        shooter.setVelocity(velocity);
    }
}
