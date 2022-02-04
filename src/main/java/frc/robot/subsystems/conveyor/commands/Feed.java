package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.BooleanSupplier;

public class Feed extends CommandBase {
    private final double power;
    private final Conveyor conveyor;
    private final BooleanSupplier activate;

    public Feed(double power, Conveyor conveyor, BooleanSupplier activate) {
        this.power = power;
        this.conveyor = conveyor;
        this.activate = activate;
    }

    @Override
    public void execute() {
        if(activate.getAsBoolean()) {
            conveyor.setPower(power);
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getCargoCount() == 0;
    }
}
