package frc.robot.subsystems.hood.commands.bits;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;

public class CheckHoodPressure extends CommandBase {
    private final Hood hood;
    private final Timer timer = new Timer();
    private double lastTime;

    public CheckHoodPressure(Hood hood) {
        this.hood = hood;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        lastTime = timer.get();
    }

    @Override
    public void execute() {
        double currentTime = timer.get();

        if(currentTime - lastTime > Constants.Hood.HOOD_PRESSURE_BIT_DELTA_TIME) {
            if(hood.isOpen()) {
                hood.close();
            } else {
                hood.open();
            }
        }

        lastTime = currentTime;
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
}
