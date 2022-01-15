package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Hood;

import java.util.function.Supplier;

public class HoodDefaultCommand extends CommandBase {
    private final Hood hood = Hood.getINSTANCE();
    private final Supplier<Boolean> mode_supplier;
    private boolean input;
    private boolean lastInput;

    public HoodDefaultCommand(Supplier<Boolean> mode_supplier) {
        this.mode_supplier = mode_supplier;
        addRequirements(hood);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        input = mode_supplier.get();
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
