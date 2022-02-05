package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.OptionalDouble;

public class Outtake extends ParallelCommandGroup {

    public Outtake(Intake intake,
                   Conveyor conveyor,
                   Shooter shooter,
                   double conveyorPower,
                   double remainingBalls) {
        addCommands(
                new ParallelCommandGroup(
                        new Feed(conveyorPower, conveyor, () -> conveyor.getCargoCount() < remainingBalls),
                        new ConditionalCommand(
                                new Shoot(shooter, () -> 8, OptionalDouble.empty()),
                                new IntakeCargo(intake, () -> true, -Constants.Intake.DEFAULT_POWER),
                                () -> conveyorPower > 0
                        )
                )
        );
    }
}
