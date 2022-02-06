package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodDefaultCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Hood.DISTANCE_FROM_TARGET_DEADBAND;

public class BasicShooting extends ParallelCommandGroup {

    public BasicShooting(Shooter shooter, Hood hood, Conveyor conveyor, DoubleSupplier distanceFromTarget) {
        addCommands(
                new HoodDefaultCommand(hood, () -> Hood.Mode.getValue(distanceFromTarget.getAsDouble() < DISTANCE_FROM_TARGET_DEADBAND)),
                new Shoot(shooter, distanceFromTarget, OptionalDouble.empty()),
                new Convey(Constants.Conveyor.DEFAULT_POWER, conveyor, () -> true)
        );
    }
}
