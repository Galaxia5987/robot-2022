package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;

public class PickUpCargo extends ParallelCommandGroup {

    public PickUpCargo(Conveyor conveyor, Intake intake, double conveyorPower, double intakePower) {
        addCommands(
                new Feed(conveyorPower, conveyor, () -> true),
                new ConditionalCommand(
                        new IntakeCargo(intake, () -> true, intakePower),
                        null,
                        () -> conveyor.getCargoCount() < 2)
        );
    }
}
