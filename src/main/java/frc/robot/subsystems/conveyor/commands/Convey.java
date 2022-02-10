package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Convey extends CommandBase {
    protected final DoubleSupplier power;
    protected final Conveyor conveyor;
    private final BooleanSupplier condition;

    public Convey(Conveyor conveyor, DoubleSupplier power, BooleanSupplier condition) {
        this.power = power;
        this.conveyor = conveyor;
        this.condition = condition;
        addRequirements(conveyor);
    }

    public Convey(Conveyor conveyor, DoubleSupplier power) {
        this(conveyor, power, () -> true);
    }

    public Convey(Conveyor conveyor, double power, BooleanSupplier condition) {
        this(conveyor, () -> power, condition);
    }

    public Convey(Conveyor conveyor, double power) {
        this(conveyor, () -> power, () -> true);
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            conveyor.setPower(power.getAsDouble());
        } else {
            conveyor.setPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }
}
