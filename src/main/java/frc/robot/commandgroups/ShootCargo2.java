package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey4;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class ShootCargo2 extends ParallelCommandGroup {

    public ShootCargo2(Shooter shooter,
                       Hood hood,
                       Conveyor conveyor,
                       Flap flap,
                       DoubleSupplier conveyorPower,
                       DoubleSupplier distanceFromTarget,
                       boolean bool) {
        DoubleSupplier setpointVelocity = () -> Shoot.getSetpointVelocity(distanceFromTarget.getAsDouble(), distanceFromTarget.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD);
        BooleanSupplier isFlywheelAtSetpoint = () -> Math.abs(setpointVelocity.getAsDouble() - shooter.getVelocity()) < SHOOTER_VELOCITY_DEADBAND.get();

        addCommands(
//                new HoodCommand(hood, () -> !conveyor.isPostFlapBeamConnected(), distanceFromTarget),
                new Convey4(conveyor, hood, () -> !conveyor.isPreFlapBeamConnected(), setpointVelocity, shooter::getVelocity, distanceFromTarget),
                new InstantCommand(flap::allowShooting),
                new Shoot(shooter, hood, distanceFromTarget, bool)
        );
    }

    public ShootCargo2(Shooter shooter,
                       Hood hood,
                       Conveyor conveyor,
                       Flap flap,
                       DoubleSupplier conveyorPower,
                       DoubleSupplier distanceFromTarget) {
        this(shooter, hood, conveyor, flap, conveyorPower, distanceFromTarget, true);
    }
}