package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

/**
 * Intakes only Cargo colored as the robot's alliance color.
 */
public class IntakeCargo extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier activate;
    private final double power;

    public IntakeCargo(Intake intake, BooleanSupplier activate, double power) {
        this.intake = intake;
        this.activate = activate;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.openRetractor();
    }

    /**
     * If ball is alliance color, ball will be sucked.
     */
    @Override
    public void execute() {
        if (activate.getAsBoolean()){
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
