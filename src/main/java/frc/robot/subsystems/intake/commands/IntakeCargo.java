package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

/**
 * Intakes cargo with a certain condition supplier.
 */
public class IntakeCargo extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier condition;
    private final double power;

    public IntakeCargo(Intake intake, BooleanSupplier condition, double power) {
        this.intake = intake;
        this.condition = condition;
        this.power = power;
        addRequirements(intake);
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
            intake.setPower(power);
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
