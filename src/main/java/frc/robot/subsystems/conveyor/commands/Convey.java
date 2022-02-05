package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.BooleanSupplier;

public class Convey extends CommandBase {
    private final double power;
    private final Conveyor conveyor;
    private final BooleanSupplier condition;

    public Convey(double power, Conveyor conveyor, BooleanSupplier condition) {
        this.power = power;
        this.conveyor = conveyor;
        this.condition = condition;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            conveyor.setPower(power);
        } else {
            conveyor.setPower(0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }
}
