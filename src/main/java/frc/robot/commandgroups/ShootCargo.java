package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.conveyor.commands.FlapDefaultCommand;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodDefaultCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

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
                      DoubleSupplier conveyorPower) {
        /*
        This boolean supplier uses a deadband for the shooter velocity by turning it into the
        ratio between the current velocity and the setpoint.
         */
        final BooleanSupplier isFlywheelAtSetpoint =
                () -> (1 - shooter.getVelocity() / Shoot.getSetpointVelocity(
                        distanceFromTarget.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND);

        addCommands(
                new HoodDefaultCommand(hood, () -> Hood.Mode.getValue(distanceFromTarget.getAsDouble() < DISTANCE_FROM_TARGET_DEADBAND)),
                new Convey(conveyorPower, conveyor, isFlywheelAtSetpoint),
                new FlapDefaultCommand(conveyor, () -> Conveyor.FlapMode.getValue(!isFlywheelAtSetpoint.getAsBoolean())),
                new Shoot(shooter, distanceFromTarget, OptionalDouble.empty())
        );
    }
}
