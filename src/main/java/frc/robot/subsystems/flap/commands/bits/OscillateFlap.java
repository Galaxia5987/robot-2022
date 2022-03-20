package frc.robot.subsystems.flap.commands.bits;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.flap.Flap;

public class OscillateFlap extends CommandBase {
    private final Flap flap;
    private final Timer timer = new Timer();

    public OscillateFlap(Flap flap) {
        this.flap = flap;
    }

    @Override
    public void initialize() {
        timer.start();
        timer.reset();
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(Constants.Hood.HOOD_PRESSURE_BIT_DELTA_TIME)) {
            flap.toggleFlap();
            timer.reset();
        }
    }
}
