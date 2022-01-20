package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

/**
 * Intakes only Cargo colored as the robot's alliance color.
 */
public class IntakeByVision extends CommandBase {
    private final Intake intake;
    private final BooleanSupplier isAllianceColor;
    private final double power;

    public IntakeByVision(Intake intake, BooleanSupplier isAllianceColor, double power) {
        this.intake = intake;
        this.isAllianceColor = isAllianceColor;
        this.power = Constants.Intake.POWER;
        addRequirements(intake);

    }

    @Override
    public void initialize() {
        intake.openSolenoid();
    }

    /**
     * If ball is alliance color, ball will be sucked.
     */
    @Override
    public void execute() {
        if (isAllianceColor.getAsBoolean())
            intake.setPower(power);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

    /**
     * Stops motor.
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

}
