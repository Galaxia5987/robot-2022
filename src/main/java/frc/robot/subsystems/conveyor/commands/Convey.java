package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.DoubleSupplier;

public class Convey extends CommandBase {

    private final Conveyor conveyor;
    private final DoubleSupplier power;

    public Convey(Conveyor conveyor, DoubleSupplier power) {
        this.conveyor = conveyor;
        this.power = power;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {}


    @Override
    public void execute() {
        conveyor.setPower(power.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
    }

}
