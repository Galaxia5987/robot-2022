package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class BasicShooting extends ParallelCommandGroup {

    public BasicShooting(Shooter shooter, Conveyor conveyor, DoubleSupplier distanceFromTarget) {
        addCommands(
                new Shoot(shooter, distanceFromTarget, OptionalDouble.empty()),
                new Feed(Constants.Conveyor.DEFAULT_POWER, conveyor, () -> true)
        );
    }
}
