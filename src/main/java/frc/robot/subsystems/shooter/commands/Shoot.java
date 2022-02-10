package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    protected final Shooter shooter;
    protected final DoubleSupplier distance;
    private final OptionalDouble power;

    /**
     * Constructor.
     *
     * @param shooter  is the shooter subsystem.
     * @param distance is the distance of the robot from the target. [rpm]
     */
    public Shoot(Shooter shooter, DoubleSupplier distance, OptionalDouble power) {
        this.shooter = shooter;
        this.distance = distance;
        this.power = power;
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, DoubleSupplier distance) {
        this(shooter, distance, OptionalDouble.empty());
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
        if (power.isEmpty()) {
            shooter.setVelocity(getSetpointVelocity(distance.getAsDouble()));
        } else {
            shooter.setPower(power.getAsDouble());
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
