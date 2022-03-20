package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LedSubsystem;

public class TestShooterVelocity extends SequentialCommandGroup {

    public TestShooterVelocity(Shooter shooter, LedSubsystem ledSubsystem) {

        addCommands(
                new ParallelRaceGroup(
                        new RunCommand(
                                () -> shooter.setVelocity(0)).withTimeout(5),
                        new RunCommand(
                                () -> System.out.println("shooter vel0: " + shooter.getVelocity()))
                ),

                new ParallelRaceGroup(
                        new RunCommand(
                                () -> shooter.setVelocity(1500)).withTimeout(5),
                        new RunCommand(
                                () -> System.out.println("shooter vel1500: " + shooter.getVelocity()))
                ),

                new ParallelRaceGroup(
                        new RunCommand(
                                () -> shooter.setVelocity(3000)).withTimeout(5),
                        new RunCommand(
                                () -> System.out.println("shooter vel3000: " + shooter.getVelocity()
                                ))
                ),

                new ParallelRaceGroup(
                        new RunCommand(
                                () -> shooter.setVelocity(0)).withTimeout(5),
                        new RunCommand(
                                () -> System.out.println("shooter vel0: " + shooter.getVelocity()))
                )
        );
    }
}
