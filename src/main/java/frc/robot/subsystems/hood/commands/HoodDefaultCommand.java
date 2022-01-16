package frc.robot.subsystems.hood.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.hood.Hood;

import java.util.function.Supplier;

public class HoodDefaultCommand extends CommandBase {
    private final Hood hood = Hood.getINSTANCE();
    private final Supplier<Boolean> modeSupplier;
    private boolean input;
    private boolean lastInput;

    public HoodDefaultCommand(Supplier<Boolean> modeSupplier) {
        this.modeSupplier = modeSupplier;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        input = modeSupplier.get();
        if (input && !lastInput) {
            hood.changeAngle();
        }
        lastInput = input;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
