package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.BooleanSupplier;

public class Convey3 extends CommandBase {
    private final Conveyor conveyor;
    private final BooleanSupplier preFlapSupplier;
    private final Timer timer = new Timer();
    private final BooleanSupplier setpointSupplier;
    boolean go = false;
    private boolean last = false;
    private boolean first = true;
    private boolean getBallToPreFlap = true;
    private Timer delayTimer = new Timer();

    public Convey3(Conveyor conveyor, BooleanSupplier preFlapSupplier, BooleanSupplier setpointSupplier) {
        this.conveyor = conveyor;
        this.preFlapSupplier = preFlapSupplier;
        this.setpointSupplier = setpointSupplier;
    }

    @Override
    public void initialize() {
        timer.stop();
        delayTimer.stop();
        if (preFlapSupplier.getAsBoolean()) {
            getBallToPreFlap = false;
        }
//        conveyor.setPower(Constants.Conveyor.DEFAULT_POWER.get());
    }

    @Override
    public void execute() {
        if (getBallToPreFlap) {
            conveyor.setPower(Constants.Conveyor.DEFAULT_POWER.get());
        } else {
//            if (delayTimer.hasElapsed(0.1))
            conveyor.setPower(0);
        }

        if (preFlapSupplier.getAsBoolean()) {
            if (!last) {
                last = true;
                getBallToPreFlap = false;
                timer.start();
                timer.reset();
                delayTimer.start();
                delayTimer.reset();
            }
        } else {
            last = false;
            getBallToPreFlap = true;
        }

        if (setpointSupplier.getAsBoolean()) {
            if (timer.hasElapsed(0.1)) { //TODO: switch timer for has reached setpoint
                getBallToPreFlap = true;
                timer.reset();
                timer.stop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        delayTimer.stop();
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
