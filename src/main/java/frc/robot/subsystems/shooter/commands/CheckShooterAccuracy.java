package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.shooter.Shooter;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class CheckShooterAccuracy extends Shoot {
    private final Shooter shooter;
    private final DoubleSupplier distance;

    public CheckShooterAccuracy(Shooter shooter, DoubleSupplier distance, OptionalDouble power) {
        super(shooter, distance, power);
        this.shooter = shooter;
        this.distance = distance;
    }

    @Override
    public void execute() {
        super.execute();

        double setpoint = Shoot.getSetpointVelocity(distance.getAsDouble());
        double currentVelocity = shooter.getVelocity();

        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putNumber("Current velocity", currentVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
