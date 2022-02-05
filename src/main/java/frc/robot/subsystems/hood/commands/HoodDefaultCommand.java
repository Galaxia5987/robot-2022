package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;

import java.util.function.Supplier;

public class HoodDefaultCommand extends CommandBase {
    private final Hood hood;
    private final Supplier<Hood.Mode> modeSupplier;

    public HoodDefaultCommand(Hood hood, Supplier<Hood.Mode> modeSupplier) {
        this.hood = hood;
        this.modeSupplier = modeSupplier;
        addRequirements(hood);
    }

    @Override
    public void execute() {
        hood.setSolenoid(modeSupplier.get());
    }
}
