package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
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
                                () -> 2).withTimeout(5),
                        new RunCommand(() -> System.out.println("Current vel for distance 2m: " + shooter.getVelocity())),
                        new RunCommand(()-> System.out.println("desired vel for distance 2m: " + Shoot.getSetpointVelocity(2,2<Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD)
                ))),

                new ParallelRaceGroup(
                        new Shoot(
                                shooter,
                                hood,
                                () -> 3.5).withTimeout(5),
                        new RunCommand(() -> System.out.println("Current vel for distance 3.5m: " + shooter.getVelocity())),
                        new RunCommand(()-> System.out.println("Desired vel for distance 3.5m: " + Shoot.getSetpointVelocity(3.5, 3.5<Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD)
                        )),

                new ParallelRaceGroup(
                        new Shoot(
                                shooter,
                                hood,
                                () -> 5).withTimeout(5),
                        new RunCommand(() -> System.out.println("Current vel for distance 5m: " + shooter.getVelocity())),
                        new RunCommand(()-> System.out.println("Desired vel for distance 5m: " + Shoot.getSetpointVelocity(5, 5<Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD))
        ))));
    }
}
