package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.flap.commands.FlapCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;

import java.util.function.DoubleSupplier;

public class PickUpCargo extends ParallelCommandGroup {

    public PickUpCargo(Conveyor conveyor, Flap flap, Intake intake, double conveyorPower, DoubleSupplier intakePower) {
        addCommands(
                new Convey(conveyor, conveyorPower),
                new FlapCommand(flap, Flap.FlapMode.STOP_CARGO),
                new IntakeCargo(intake, () -> true, intakePower)
        );
    }
}
