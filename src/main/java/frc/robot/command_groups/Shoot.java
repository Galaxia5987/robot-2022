package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.PrepareShooter;

import java.util.function.DoubleSupplier;

public class Shoot extends ParallelCommandGroup {
    private Shooter shooter;
    private DoubleSupplier distance;

    public Shoot(Shooter shooter, DoubleSupplier distance) {
        this.shooter = shooter;
        this.distance = distance;
        /*
        Once the rest of the robot is operational, another command will
        be added to feed the balls to the shooter.
         */
        addCommands(
                new PrepareShooter(shooter, () -> shooter.getSetpointVelocity(distance.getAsDouble()))
        );
    }
}
