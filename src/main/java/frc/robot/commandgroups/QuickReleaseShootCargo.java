package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.QuickReleaseConvey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.commands.HoodCommand;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.function.DoubleSupplier;

public class QuickReleaseShootCargo extends ParallelCommandGroup {

    public QuickReleaseShootCargo(Shooter shooter,
                                  Hood hood,
                                  Conveyor conveyor,
                                  Flap flap,
                                  DoubleSupplier conveyorPower,
                                  DoubleSupplier distanceFromTarget,
                                  boolean bool) {
        DoubleSupplier setpointVelocity = () -> Shoot.getSetpointVelocity(distanceFromTarget.getAsDouble());

        addCommands(
                new HoodCommand(hood, () -> !conveyor.isPostFlapBeamConnected(), RobotContainer.hardCodedVelocity ? () -> 3.6 : distanceFromTarget),
                new QuickReleaseConvey(conveyor, () -> !conveyor.isPreFlapBeamConnected(), RobotContainer.hardCodedVelocity ? () -> Constants.Shooter.TARMAC_VELOCITY : setpointVelocity, shooter::getVelocity),
                new InstantCommand(flap::allowShooting),
                new Shoot(shooter, hood, distanceFromTarget, bool)
        );
    }

    public QuickReleaseShootCargo(Shooter shooter,
                                  Hood hood,
                                  Conveyor conveyor,
                                  Flap flap,
                                  DoubleSupplier conveyorPower,
                                  DoubleSupplier distanceFromTarget) {
        this(shooter, hood, conveyor, flap, conveyorPower, distanceFromTarget, true);
    }
}
