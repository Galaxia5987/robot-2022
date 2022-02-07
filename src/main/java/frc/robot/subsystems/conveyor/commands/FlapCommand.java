package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.Supplier;

public class FlapCommand extends CommandBase {
    private final Conveyor conveyor;
    private final Supplier<Conveyor.FlapMode> flapMode;

    public FlapCommand(Conveyor conveyor, Supplier<Conveyor.FlapMode> flapMode) {
        this.conveyor = conveyor;
        this.flapMode = flapMode;
        addRequirements(conveyor);
    }

    public FlapCommand(Conveyor conveyor, Conveyor.FlapMode flapMode) {
        this(conveyor, () -> flapMode);
        execute();
        cancel();
    }

    @Override
    public void execute() {
        if (flapMode.get().mode) {
            conveyor.openFlap();
        } else {
            conveyor.closeFlap();
        }
    }

    @Override
    public void end(boolean interrupted) {
        conveyor.setFlapMode(Conveyor.FlapMode.Closed);
    }
}
