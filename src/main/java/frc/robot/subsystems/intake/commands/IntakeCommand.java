package frc.robot.subsystems.intake.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

public class IntakeCommand extends CommandBase {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);

    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        intake.setPower(Constants.Intake.POWER);
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
