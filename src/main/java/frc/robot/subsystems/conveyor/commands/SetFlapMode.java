package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;

public class SetFlapMode extends SequentialCommandGroup {

    public SetFlapMode(Conveyor.FlapMode flapMode, Conveyor conveyor) {
        addCommands(
                new InstantCommand(() -> {
                    if (flapMode.mode)
                        conveyor.openFlap();
                    else
                        conveyor.closeFlap();
                    }
                ));
    }
}
