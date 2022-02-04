package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.conveyor.commands.SetFlapMode;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.Utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.CommandGroups.SHOOTER_VELOCITY_DEADBAND;

public class ShootCargo extends ParallelCommandGroup {

    public ShootCargo(Shooter shooter,
                      Conveyor conveyor,
                      DoubleSupplier distanceFromTarget,
                      double conveyorPower) {
        final BooleanSupplier isAtSetpoint =
                () -> Utils.deadband(
                        shooter.getVelocity() / Shoot.getSetpointVelocity(
                                distanceFromTarget.getAsDouble()) - 1, SHOOTER_VELOCITY_DEADBAND) == 0;

        addCommands(
                new ConditionalCommand(
                        new SetFlapMode(conveyor, Conveyor.FlapMode.Open),
                        new SetFlapMode(conveyor, Conveyor.FlapMode.Closed),
                        isAtSetpoint),
                new ConditionalCommand(
                        new Feed(conveyorPower, conveyor),
                        null,
                        isAtSetpoint),
                new Shoot(shooter, distanceFromTarget)
        );
    }
}
