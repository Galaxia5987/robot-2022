package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

/**
 * default intake command just sucks ball
 */
public class DefaultIntake extends CommandBase {
    private final Intake intake;

    public DefaultIntake(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    /**
     * set intake motor to an already decided default power
     */
    @Override
    public void initialize() {
        intake.setPower(Constants.Intake.POWER);
        intake.openSolenoid();
    }

    /**
     * set intake motor power to 0 stopping it
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
        intake.closeSolenoid();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
