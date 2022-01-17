package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class PrepareShooter extends CommandBase {
    private final Shooter shooter;
    private final DoubleSupplier velocitySetpoint;

    /**
     * Constructor.
     *
     * @param shooter          is the shooter subsystem.
     * @param velocitySetpoint is the setpoint for the shooter wheel. [rps]
     */
    public PrepareShooter(Shooter shooter, DoubleSupplier velocitySetpoint) {
        this.shooter = shooter;
        this.velocitySetpoint = velocitySetpoint;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setVelocity(velocitySetpoint.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }
}
