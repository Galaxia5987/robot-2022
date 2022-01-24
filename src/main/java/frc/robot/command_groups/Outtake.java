package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.function.DoubleSupplier;

public class Outtake extends ParallelCommandGroup {

    public Outtake(Shooter shooter, DoubleSupplier distance) {
        /*
        Once the rest of the robot is operational, another command will
        be added to feed the balls to the shooter.
         */
        addCommands(
                new Shoot(shooter, () -> shooter.getSetpointVelocity(distance.getAsDouble()))
        );
    }
}
