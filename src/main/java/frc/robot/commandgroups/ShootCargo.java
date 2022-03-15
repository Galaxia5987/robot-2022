package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey3;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ShootCargo extends ParallelCommandGroup {

    public ShootCargo(Shooter shooter,
                      Hood hood,
                      Conveyor conveyor,
                      Flap flap,
                      DoubleSupplier conveyorPower,
                      DoubleSupplier distanceFromTarget,
                      boolean bool, BooleanSupplier hasTarget, DoubleSupplier odometryDistance) {
        DoubleSupplier setpointVelocity = () -> Shoot.getSetpointVelocity(distanceFromTarget.getAsDouble());

        addCommands(
                new HoodCommand(hood, () -> !conveyor.isPostFlapBeamConnected(), distanceFromTarget),
                new Convey3(conveyor, () -> !conveyor.isPreFlapBeamConnected(), setpointVelocity, shooter::getVelocity),
                new InstantCommand(flap::allowShooting),
                new Shoot(shooter, hood, distanceFromTarget, bool, hasTarget, odometryDistance)
        );
    }

    public ShootCargo(Shooter shooter,
                      Hood hood,
                      Conveyor conveyor,
                      Flap flap,
                      DoubleSupplier conveyorPower,
                      DoubleSupplier distanceFromTarget, BooleanSupplier hasTarget, DoubleSupplier odometryDistance) {
        this(shooter, hood, conveyor, flap, conveyorPower, distanceFromTarget, true, hasTarget, odometryDistance);
    }
}
