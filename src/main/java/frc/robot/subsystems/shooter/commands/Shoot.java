package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import webapp.FireLog;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    protected final Shooter shooter;
    protected final DoubleSupplier setpointVelocity;
    private final OptionalDouble power;

    public Shoot(Shooter shooter, double power) {
        this.shooter = shooter;
        this.setpointVelocity = () -> 8;
        this.power = OptionalDouble.of(power);
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, DoubleSupplier setpointVelocity) {
        this.shooter = shooter;
        this.setpointVelocity = setpointVelocity;
        this.power = OptionalDouble.empty();
        addRequirements(shooter);
    }

    /**
     * Calculates the velocity setpoint according to the distance from the target.
     * Once the data from the shooter is acquired this function will be changed.
     *
     * @param distance is the distance from the target. [m]
     * @return the setpoint from the shooter util (for now). [rpm]
     */
    public static double getSetpointVelocity(double distance) {
        return distance;
    }

    @Override
    public void execute() {
        if (power.isEmpty()) {
            shooter.setVelocity(setpointVelocity.getAsDouble());
        } else {
            shooter.setPower(power.getAsDouble());
        }

        SmartDashboard.putNumber("something_velocity", shooter.getVelocity()); // Used in the shooter util
        FireLog.log("Shooter velocity", shooter.getVelocity());
        FireLog.log("Shooter setpoint", getSetpointVelocity(setpointVelocity.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
