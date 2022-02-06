package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class CheckShooterAccuracy extends Shoot {
    private final Shooter shooter;
    private final DoubleSupplier distance;
    private final Timer timer = new Timer();

    public CheckShooterAccuracy(Shooter shooter, DoubleSupplier distance, OptionalDouble power) {
        super(shooter, distance, power);
        this.shooter = shooter;
        this.distance = distance;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        super.execute();

        double setpoint = Shoot.getSetpointVelocity(distance.getAsDouble());
        double currentVelocity = shooter.getVelocity();

        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putNumber("Current velocity", currentVelocity);
        SmartDashboard.putNumber("Time", timer.get());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        double finalTime = timer.get();
        if (finalTime < Constants.Shooter.RECOMMENDED_ACCELERATION_TIME) {
            System.out.println("The shooter worked quickly :)");
        } else {
            System.out.println("The shooter is slow :(");
        }
    }
}
