package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;

public class ConveyCargo extends CommandBase {
    private final Conveyor conveyor;
    private final Timer timer = new Timer();
    private boolean stopForSecondBall = false;
    private boolean lastPreFlapObservation = false;

    public ConveyCargo(Conveyor conveyor) {
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.stop();
    }

    @Override
    public void execute() {
        if (RobotContainer.Suppliers.preFlapBlocked.getAsBoolean()) {
            lastPreFlapObservation = true;
        } else if (lastPreFlapObservation) {
            stopForSecondBall = true;
        }
        if (RobotContainer.Suppliers.postFlapBlocked.getAsBoolean()) {
            stopForSecondBall = true;
        }
        if (stopForSecondBall) {
            if (!RobotContainer.Suppliers.preFlapBlocked.getAsBoolean()) {
                conveyor.setPower(Constants.Conveyor.SHOOT_POWER);
            } else {
                conveyor.setPower(0);
                timer.start();
            }
            if (timer.hasElapsed(0.3)) {
                conveyor.setPower(Constants.Conveyor.SHOOT_POWER);
            }
        } else {
            conveyor.setPower(Constants.Conveyor.SHOOT_POWER);
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
