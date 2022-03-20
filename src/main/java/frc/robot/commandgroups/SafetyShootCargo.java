package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.subsystems.shooter.commands.WarmUpShooter;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class SafetyShootCargo extends ParallelCommandGroup {

    public SafetyShootCargo(Shooter shooter, Conveyor conveyor, Flap flap, Hood hood, DoubleSupplier distanceFromTarget) {
        addCommands(
                new InstantCommand(flap::allowShooting),
                new InstantCommand(() -> hood.setSolenoid(Hood.Mode.getValue(distanceFromTarget.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD))),
                new WarmUpShooter(shooter, distanceFromTarget),
                new Convey(conveyor, Constants.Conveyor.SHOOT_POWER,
                        () -> Utils.conventionalDeadband(
                                shooter.getVelocity() - Shoot.getSetpointVelocity(
                                        distanceFromTarget.getAsDouble()), Constants.Shooter.SHOOTER_VELOCITY_DEADBAND.get()) == 0)
        );
    }
}
