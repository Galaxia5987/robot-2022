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
     * @param shooter          is the shooter subsystem.
     * @param distance is the distance of the robot from the target. [rpm]
     */
    public Shoot(Shooter shooter, DoubleSupplier distance) {
        this.shooter = shooter;
        this.distance = distance;
        addRequirements(shooter);
    }

    /**
     * Calculates the velocity setpoint according to the distance from the target.
     * Once the data from the shooter is acquired this function will be changed.
     *
     * @param distance is the distance from the target. [m]
     * @return 15. [rpm]
     */
    public static double getSetpointVelocity(double distance) {
        return 100;
    }

    @Override
    public void execute() {
        shooter.setVelocity(getSetpointVelocity(distance.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
