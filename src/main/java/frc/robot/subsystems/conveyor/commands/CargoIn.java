package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

public class CargoIn extends CommandBase {
    private final Conveyor conveyor;
    private final double power;

    public CargoIn(Conveyor conveyor, double power) {
        this.conveyor = conveyor;
        this.power = power;
        addRequirements(conveyor);
    }

    @Override
    public void execute() {
        conveyor.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return conveyor.getCargoCount() >= Constants.Conveyor.MAX_CARGO_AMOUNT;
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setPower(0);
    }
}