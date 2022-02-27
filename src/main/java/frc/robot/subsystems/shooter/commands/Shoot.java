package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterDataModule;
import webapp.FireLog;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    protected final Shooter shooter;
    protected final Hood hood;
    protected final DoubleSupplier distance;
    private final OptionalDouble power;

    public Shoot(Shooter shooter, Hood hood, double power) {
        this.shooter = shooter;
        this.hood = hood;
        this.distance = () -> 8;
        this.power = OptionalDouble.of(power);
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, Hood hood, DoubleSupplier distance) {
        this.shooter = shooter;
        this.hood = hood;
        this.distance = distance;
        this.power = OptionalDouble.empty();
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
        ShooterDataModule data = ShooterDataModule.getDataModule(distance);
        return data.getShooterVelocity();
    }

    @Override
    public void execute() {
        if (power.isEmpty()) {
            double distance = this.distance.getAsDouble();
            shooter.setVelocity(4300);
            System.out.println("Distance: " + distance + ", Velocity: " + getSetpointVelocity(distance, hood.isOpen()));
//            shooter.setVelocity(getSetpointVelocity(distance.getAsDouble()));
        } else {
            shooter.setPower(power.getAsDouble());
        }

        SmartDashboard.putNumber("something_velocity", shooter.getVelocity());
        FireLog.log("Shooter velocity", shooter.getVelocity());
        FireLog.log("Shooter setpoint", getSetpointVelocity(distance.getAsDouble()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
