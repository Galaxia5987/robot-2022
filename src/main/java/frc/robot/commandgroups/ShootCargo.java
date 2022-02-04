package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.CommandGroups.SHOOTER_VELOCITY_DEADBAND;

public class ShootCargo extends ParallelCommandGroup {

    public ShootCargo(Shooter shooter, Conveyor conveyor, DoubleSupplier distanceFromTarget, double conveyorPower) {
        addCommands(
                new ConditionalCommand(
                        new Feed(conveyorPower, conveyor),
                        null,
                        () -> Utils.deadband(shooter.getVelocity() / Shoot.getSetpointVelocity(distanceFromTarget.getAsDouble()), SHOOTER_VELOCITY_DEADBAND) == 0),
                new Shoot(shooter, distanceFromTarget)
        );
    }
}
