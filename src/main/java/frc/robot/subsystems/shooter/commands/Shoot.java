package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Shoot extends SequentialCommandGroup {
    private final double required_velocity;

    public Shoot(double required_velocity) {
        this.required_velocity = required_velocity;

        /*
        Once the rest of the robot is operational, another command will
        be added to feed the balls to the shooter.
         */
        addCommands(
                new PrepareShooter(required_velocity)
        );
    }
}
