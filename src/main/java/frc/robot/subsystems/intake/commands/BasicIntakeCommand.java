package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

/**
 * Default intake command just sucks ball.
 */
public class BasicIntakeCommand extends CommandBase {
    private final Intake intake;

    public BasicIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    /**
     * Set intake motor to an already decided default power.
     */
    @Override
    public void initialize() {
        intake.setPower(Constants.Intake.POWER);
        intake.openSolenoid();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Set intake motor power to 0% stopping it.
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }


}
