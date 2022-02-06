package frc.robot.subsystems.intake.commands.bits;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;


public class RetractorTest extends CommandBase {
    private final Timer timer = new Timer();
    private final Intake intake;
    private final int numOfRuns;
    private double previousTimeStamp = 0;
    private boolean previousState;
    private int counter = 0;

    public RetractorTest(int numOfRuns, Intake intake) {
        this.numOfRuns = numOfRuns;
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        previousState = intake.getRetractorState();
    }

    @Override
    public void execute() {
        double currentTimeStamp = timer.get();
        if (currentTimeStamp - previousTimeStamp > Constants.Intake.TIME_BETWEEN_RUNS) {
            intake.toggleRetractor();
            boolean currentState = intake.getRetractorState();
            previousTimeStamp = currentTimeStamp;
            if (previousState != currentState)
                System.out.println("run number " + counter + " worked");
            previousState = currentState;
            counter++;
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
