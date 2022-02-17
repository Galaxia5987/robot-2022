package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.flap.commands.FlapCommand;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD;
import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class ShootCargo extends ParallelCommandGroup {

    public ShootCargo(Shooter shooter,
                      Hood hood,
                      Conveyor conveyor,
                      Flap flap,
                      DoubleSupplier conveyorPower,
                      DoubleSupplier distanceFromTarget) {
        /*
        This boolean supplier uses a deadband for the shooter velocity by turning it into the
        ratio between the current velocity and the setpoint.
         */
        this(shooter, hood, conveyor, flap, conveyorPower, distanceFromTarget, Shoot.getSetpointVelocity(distanceFromTarget.getAsDouble()));
    }

    public ShootCargo(Shooter shooter,
                      Hood hood,
                      Conveyor conveyor,
                      Flap flap,
                      DoubleSupplier conveyorPower,
                      DoubleSupplier distanceFromTarget,
                      double velocity) {
        final BooleanSupplier isFlywheelAtSetpoint =
                () -> Math.abs(velocity - shooter.getVelocity()) < SHOOTER_VELOCITY_DEADBAND;

        addCommands(
                new HoodCommand(hood, () -> Hood.Mode.getValue(distanceFromTarget.getAsDouble() < DISTANCE_FROM_TARGET_THRESHOLD)),
                new Convey(conveyor, conveyorPower, isFlywheelAtSetpoint),
                new FlapCommand(flap, () -> Flap.FlapMode.getValue(!isFlywheelAtSetpoint.getAsBoolean())),
                new Shoot(shooter, distanceFromTarget)
        );
    }
}
