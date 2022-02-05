package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.Supplier;

public class SetFlapMode extends CommandBase {
    private final Conveyor conveyor;
    private final Supplier<Conveyor.FlapMode> flapMode;

    public SetFlapMode(Conveyor conveyor, Supplier<Conveyor.FlapMode> flapMode) {
        this.conveyor = conveyor;
        this.flapMode = flapMode;

        addRequirements(conveyor);
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
