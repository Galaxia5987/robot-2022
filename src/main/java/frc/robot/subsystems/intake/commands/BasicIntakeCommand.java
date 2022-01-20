package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

/**
 * intakes balls.
 */
public class BasicIntakeCommand extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier isAllianceColor;
    private final double power;

    public BasicIntakeCommand(Intake intake, BooleanSupplier isAllianceColor, double power) {
        this.intake = intake;
        this.isAllianceColor = isAllianceColor;
        this.power = Constants.Intake.POWER;
        addRequirements(intake);
    }

    /**
     * Set intake motor to an already decided default power.
     */
    @Override
    public void initialize() {
        intake.openSolenoid();
        if (isAllianceColor.getAsBoolean())
            intake.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    /**
     * Set intake motor power to 0% stopping it.
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
}
