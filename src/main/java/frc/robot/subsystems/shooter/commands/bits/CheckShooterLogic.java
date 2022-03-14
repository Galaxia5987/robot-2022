package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

public class CheckShooterLogic extends SequentialCommandGroup {

    public CheckShooterLogic(Shooter shooter,
                             Hood hood) {

        addCommands(
                new ParallelRaceGroup(
                        new Shoot(
                                shooter,
                                hood,
                                () -> 2),
                        new RunCommand(() -> System.out.println("vel for distance 2m: " + shooter.getVelocity()))
                ).withTimeout(5),

                new ParallelRaceGroup(
                        new Shoot(
                                shooter,
                                hood,
                                () -> 3),
                        new RunCommand(() -> System.out.println("vel for distance 3m: " + shooter.getVelocity())
                        ).withTimeout(5)),

                new ParallelRaceGroup(
                        new Shoot(
                                shooter,
                                hood,
                                () -> 5),
                        new RunCommand(() -> System.out.println("vel for distance 5m: " + shooter.getVelocity())
                        ).withTimeout(5)));

    }
}
