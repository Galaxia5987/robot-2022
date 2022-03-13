package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BackAndShootCargoSort extends SequentialCommandGroup {
    public BackAndShootCargoSort(Shooter shooter,
                                 Hood hood,
                                 Conveyor conveyor,
                                 Flap flap,
                                 DoubleSupplier conveyorPower,
                                 DoubleSupplier distanceFromTarget, BooleanSupplier hasTarget, DoubleSupplier odomDistance) {
        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075).withInterrupt(conveyor::isPreFlapBeamConnected),
                new ShootCargo2(shooter, hood, conveyor, flap, conveyorPower, distanceFromTarget, hasTarget, odomDistance));
    }
}