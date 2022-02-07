package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;

import java.util.function.Supplier;

public class HoodCommand extends CommandBase {
    private final Hood hood;
    private final Supplier<Hood.Mode> modeSupplier;

    public HoodCommand(Hood hood, Supplier<Hood.Mode> modeSupplier) {
        this.hood = hood;
        this.modeSupplier = modeSupplier;
        addRequirements(hood);
    }

    public HoodCommand(Hood hood, Hood.Mode mode) {
        this(hood, () -> mode);
        execute();
        cancel();
    }

    @Override
    public void execute() {
        hood.setSolenoid(modeSupplier.get());
    }
}
