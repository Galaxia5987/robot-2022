package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier distance;

    /**
     * Constructor.
     *
     * @param shooter  is the shooter subsystem.
     * @param distance is the distance of the robot from the target. [m]
     */
    public Shoot(Shooter shooter, DoubleSupplier distance) {
        this.shooter = shooter;
        this.distance = distance;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(Shooter.getSetpointVelocity(distance.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
