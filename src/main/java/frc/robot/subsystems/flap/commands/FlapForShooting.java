package frc.robot.subsystems.flap.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.flap.Flap;

import java.util.function.BooleanSupplier;

public class FlapForShooting extends CommandBase {
    private final Flap flap;
    private final BooleanSupplier hasReachedSetpoint;
    private final BooleanSupplier hasSensedObject;
    private final Timer timer = new Timer();
    private boolean targetMode = Flap.FlapMode.Open.mode;

    public FlapForShooting(Flap flap, BooleanSupplier hasReachedSetpoint, BooleanSupplier hasSensedObject) {
        this.flap = flap;
        this.hasReachedSetpoint = hasReachedSetpoint;
        this.hasSensedObject = hasSensedObject;

        addRequirements(flap);
    }

    @Override
    public void initialize() {
        flap.blockShooter();
    }

    @Override
    public void execute() {
        if (hasSensedObject.getAsBoolean() && hasReachedSetpoint.getAsBoolean()) {
            flap.allowShooting();
            timer.start();
        } else if (timer.hasElapsed(Constants.Flap.FLAP_DELAY)) {
            flap.blockShooter();
            timer.stop();
            timer.reset();
        }

    }
}
