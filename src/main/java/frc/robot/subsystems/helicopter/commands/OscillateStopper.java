package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.helicopter.Helicopter;

public class OscillateStopper extends CommandBase {
    private final Helicopter helicopter;
    private final Timer timer = new Timer();

    public OscillateStopper(Helicopter helicopter) {
        this.helicopter = helicopter;
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(Constants.Helicopter.OSCILLATION_DELTA_TIME)) {
            helicopter.toggleStopper();
            timer.reset();
        }
    }
}
