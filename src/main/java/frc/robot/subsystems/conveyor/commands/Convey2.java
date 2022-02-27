package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class Convey2 extends CommandBase {
    protected final DoubleSupplier power;
    protected final Conveyor conveyor;
    private final DoubleSupplier setpointSupplier;
    private final DoubleSupplier shooterVelocitySupplier;
    private double maximalProximity = 0;
    private double setpoint;

    public Convey2(Conveyor conveyor, DoubleSupplier power, DoubleSupplier setpoint, DoubleSupplier shooterVelocitySupplier) {
        this.power = power;
        this.conveyor = conveyor;
        this.setpointSupplier = setpoint;
        this.shooterVelocitySupplier = shooterVelocitySupplier;
        addRequirements(conveyor);
    }


    @Override
    public void initialize() {
        maximalProximity = 0;
        setpoint = setpointSupplier.getAsDouble();
    }

    @Override
    public void execute() {
        double powerValue = power.getAsDouble();
        if (Math.abs(setpoint - shooterVelocitySupplier.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get()) {
            SmartDashboard.putNumber("Conveyor power", powerValue);
            conveyor.setPower(power.getAsDouble());
        } else {
            conveyor.setPower(0);
        }
        SmartDashboard.putNumber("Conveyor p", conveyor.getPower());

        maximalProximity = Math.max(maximalProximity, conveyor.getProximityValue());
        System.out.println("Maximal proximity " + maximalProximity);
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }
}
