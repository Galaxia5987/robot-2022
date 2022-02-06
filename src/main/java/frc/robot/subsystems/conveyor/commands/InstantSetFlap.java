package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.conveyor.Conveyor;

public class InstantSetFlap extends InstantCommand {
    private final Conveyor conveyor;
    private final Conveyor.FlapMode flapMode;

    public InstantSetFlap( Conveyor conveyor, Conveyor.FlapMode flapMode) {
        this.flapMode = flapMode;
        this.conveyor = conveyor;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        conveyor.setFlapMode(flapMode);
    }
}
