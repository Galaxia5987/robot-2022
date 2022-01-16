package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

/**
 * default intake but only sucks alliance colored balls
 */
public class IntakeByVision extends CommandBase {
    Intake intake;
    BooleanSupplier isAllianceColor;

    public IntakeByVision(Intake intake, BooleanSupplier isAllianceColor) {
        this.intake = intake;
        this.isAllianceColor = isAllianceColor;
        addRequirements(intake);

    }

    @Override
    public void initialize() {
        intake.openSolenoid();
    }

    /**
     * if ball is alliance color, ball will be sucked
     */
    @Override
    public void execute() {
        if (isAllianceColor.getAsBoolean())
            intake.setPower(Constants.Intake.POWER);
        else
            intake.setPower(0);
    }

    /**
     * stops motor
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
        intake.closeSolenoid();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
