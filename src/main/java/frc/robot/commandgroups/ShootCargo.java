package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.ConveyToShooter;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodCommand;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class ShootCargo extends ParallelCommandGroup {

    public ShootCargo(Shooter shooter,
                      Hood hood,
                      Conveyor conveyor,
                      Flap flap,
                      DoubleSupplier distanceFromTarget) {
        addCommands(
                new HoodCommand(hood),
//                new ConveyToShooter(conveyor, () -> !conveyor.isPreFlapBeamConnected(), shooter::getVelocity),
                new ConveyToShooter(conveyor, () -> !conveyor.isPreFlapBeamConnected(), shooter::getVelocity),
                new InstantCommand(flap::allowShooting)
//                new Shoot(shooter, hood, distanceFromTarget)
        );
    }
}
