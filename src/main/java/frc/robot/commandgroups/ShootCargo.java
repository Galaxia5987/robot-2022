package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.flap.commands.FlapForShooting;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class ShootCargo extends ParallelCommandGroup {

    public ShootCargo(Shooter shooter,
                      Hood hood,
                      Conveyor conveyor,
                      Flap flap,
                      DoubleSupplier conveyorPower,
                      DoubleSupplier distanceFromTarget) {
        DoubleSupplier setpointVelocity = () -> Shoot.getSetpointVelocity(distanceFromTarget.getAsDouble(), hood.isOpen());
        BooleanSupplier isFlywheelAtSetpoint = () -> Math.abs(setpointVelocity.getAsDouble() - shooter.getVelocity()) < SHOOTER_VELOCITY_DEADBAND.get();

        addCommands(
                new HoodCommand(hood, () -> !conveyor.isPostFlapBeamConnected(), distanceFromTarget),
                new Convey(conveyor, conveyorPower, isFlywheelAtSetpoint),
                new ParallelCommandGroup().perpetually(),
                new FlapForShooting(flap, isFlywheelAtSetpoint, () -> !conveyor.isPreFlapBeamConnected()),
                new Shoot(shooter, hood, distanceFromTarget)
        );
    }
}
