package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.AngleChanger;

import java.util.function.Supplier;

public class AngleChangerDefaultCommand extends CommandBase {
    private final AngleChanger angleChanger = AngleChanger.getINSTANCE();
    private final Supplier<Boolean> mode_supplier;
    private boolean input;
    private boolean lastInput;

    public AngleChangerDefaultCommand(Supplier<Boolean> mode_supplier) {
        this.mode_supplier = mode_supplier;
        addRequirements(angleChanger);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        input = mode_supplier.get();
        if (input && !lastInput) {
            angleChanger.changeAngle();
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
