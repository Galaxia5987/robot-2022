package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.conveyor.commands.FlapDefaultCommand;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodDefaultCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.Utils;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.CommandGroups.SHOOTER_VELOCITY_DEADBAND;
import static frc.robot.Constants.Hood.DISTANCE_FROM_TARGET_DEADBAND;

public class ShootCargo extends ParallelCommandGroup {

    public ShootCargo(Shooter shooter,
                      Hood hood,
                      Conveyor conveyor,
                      DoubleSupplier distanceFromTarget,
                      double conveyorPower) {
        final BooleanSupplier isFlywheelAtSetpoint =
                () -> Utils.deadband(
                        1 - shooter.getVelocity() / Shoot.getSetpointVelocity(
                                distanceFromTarget.getAsDouble()), SHOOTER_VELOCITY_DEADBAND) == 0;

        addCommands(
                new HoodDefaultCommand(hood, () -> Hood.Mode.getValue(distanceFromTarget.getAsDouble() < DISTANCE_FROM_TARGET_DEADBAND)),
                new Feed(conveyorPower, conveyor, isFlywheelAtSetpoint),
                new FlapDefaultCommand(conveyor, () -> Conveyor.FlapMode.getValue(isFlywheelAtSetpoint.getAsBoolean())),
                new Shoot(shooter, distanceFromTarget, OptionalDouble.empty())
        );
    }
}
