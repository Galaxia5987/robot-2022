package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.DoubleSupplier;

public class BackAndShootCargo extends SequentialCommandGroup {
    public BackAndShootCargo(Shooter shooter,
                             Hood hood,
                             Conveyor conveyor,
                             Flap flap,
                             DoubleSupplier conveyorPower,
                             DoubleSupplier distanceFromTarget) {
        addCommands(new InstantCommand(() -> RobotContainer.cachedSetpoint = RobotContainer.setpointSupplier.getAsDouble()));
        addCommands(new InstantCommand(() -> RobotContainer.cachedDistance = RobotContainer.distanceSupplier.getAsDouble()));
        addCommands(new InstantCommand(() -> RobotContainer.odometryCachedSetpoint = RobotContainer.odometrySetpointSupplier.getAsDouble()));
        addCommands(new InstantCommand(() -> RobotContainer.odometryCachedDistance = RobotContainer.odometryDistanceSupplier.getAsDouble()));
        addCommands(new InstantCommand(() -> RobotContainer.cachedHasTarget = !RobotContainer.playWithoutVision && RobotContainer.hasTarget.getAsBoolean()));

        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075).withInterrupt(conveyor::isPreFlapBeamConnected),
                new ShootCargo(shooter, hood, conveyor, flap, conveyorPower, distanceFromTarget));


    }
}