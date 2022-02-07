package frc.robot.subsystems.intake.commands.bits;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;


public class RetractorTest extends CommandBase {
    private final Timer timer = new Timer();
    private final Intake intake;
    private final int numOfRuns;
    private boolean previousState;
    private int counter = 0;

    public RetractorTest(Intake intake, int numOfRuns) {
        this.numOfRuns = numOfRuns;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        intake.toggleRetractor();
        previousState = intake.getRetractorState();
    }

    @Override
    public void execute() {
        if (timer.advanceIfElapsed(Constants.Intake.TIME_BETWEEN_RUNS)) {
            intake.toggleRetractor();
            boolean currentState = intake.getRetractorState();

            if (previousState != currentState)
                System.out.println("run number " + counter + " worked");

            previousState = currentState;
            counter++;
            timer.reset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.closeRetractor();
    }

    @Override
    public boolean isFinished() {
        return numOfRuns == counter;
    }
}
