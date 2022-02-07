package frc.robot.subsystems.flap.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.flap.Flap;

import java.util.function.Supplier;

public class FlapCommand extends CommandBase {
    private final Flap flap;
    private final Supplier<Flap.FlapMode> flapMode;
    private boolean isInstant = false;

    public FlapCommand(Flap flap, Supplier<Flap.FlapMode> flapMode) {
        this.flap = flap;
        this.flapMode = flapMode;
        addRequirements(flap);
    }

    public FlapCommand(Flap flap, Flap.FlapMode flapMode) {
        this(flap, () -> flapMode);
        isInstant = true;
    }

    @Override
    public void initialize() {
        if(isInstant) {
            execute();
            cancel();
        }
    }

    @Override
    public void execute() {
        if (flapMode.get().mode) {
            flap.openFlap();
        } else {
            flap.closeFlap();
        }
    }

    @Override
    public void end(boolean interrupted) {
        flap.setFlapMode(Flap.FlapMode.Closed);
    }
}
