package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Intakes cargo with a certain condition supplier.
 */
public class IntakeCargo extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier condition;
    private final DoubleSupplier power;

    public IntakeCargo(Intake intake, BooleanSupplier condition, DoubleSupplier power) {
        this.intake = intake;
        this.condition = condition;
        this.power = power;
        addRequirements(intake);
    }

    public IntakeCargo(Intake intake, DoubleSupplier power) {
        this(intake, () -> true, power);
    }

    @Override
    public void initialize() {
        intake.openRetractor();
    }

    /**
     * If activation boolean supplier returns true, activate the intake.
     */
    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            intake.setPower(power.getAsDouble());
        } else {
            intake.setPower(0);
        }
    }

    /**
     * Stops motor.
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

}
