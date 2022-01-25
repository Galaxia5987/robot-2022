package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    private final DoubleSupplier distance;
    private final Shooter shooter;

    public Shoot(DoubleSupplier distance, Shooter shooter) {
        this.distance = distance;
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(shooter.getSetpointVelocity(distance.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
