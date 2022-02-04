package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.conveyor.Conveyor;

public class SetFlapMode extends InstantCommand {
    private final Conveyor conveyor;
    private final Conveyor.FlapMode flapMode;

    public SetFlapMode(Conveyor conveyor, Conveyor.FlapMode flapMode) {
        this.conveyor = conveyor;
        this.flapMode = flapMode;

        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        if (flapMode.mode) {
            conveyor.openFlap();
        } else {
            conveyor.closeFlap();
        }
    }
}
