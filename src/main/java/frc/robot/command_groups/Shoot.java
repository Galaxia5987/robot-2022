package frc.robot.command_groups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.PrepareShooter;

public class Shoot extends SequentialCommandGroup {
    public Shoot(Shooter shooter, double distance) {
        /*
        Once the rest of the robot is operational, another command will
        be added to feed the balls to the shooter.
         */
        addCommands(
                new PrepareShooter(shooter, getSetpointVelocity(distance))
        );
    }

    /**
     * Calculates the velocity setpoint according to the distance from the target.
     *
     * @param distance is the distance from the target. [m]
     * @return 15. [rps]
     */
    private double getSetpointVelocity(double distance) {
        return 15;
    }
}
