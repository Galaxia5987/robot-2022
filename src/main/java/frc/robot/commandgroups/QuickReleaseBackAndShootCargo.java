package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class QuickReleaseBackAndShootCargo extends SequentialCommandGroup {
    public QuickReleaseBackAndShootCargo(Shooter shooter,
                                         Hood hood,
                                         Conveyor conveyor,
                                         Flap flap,
                                         DoubleSupplier distanceFromTarget) {
    }
}