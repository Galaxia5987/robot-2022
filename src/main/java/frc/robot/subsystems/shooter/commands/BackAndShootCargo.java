package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class BackAndShootCargo extends SequentialCommandGroup {
    public BackAndShootCargo(Shooter shooter,
                             Hood hood,
                             Conveyor conveyor,
                             Flap flap,
                             DoubleSupplier distanceFromTarget) {

    }
}