package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.testing.HelpfulZeroing;
import frc.robot.subsystems.drivetrain.commands.testing.TurnToRandomAngles;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class RunAllBits extends SequentialCommandGroup {

    public RunAllBits(SwerveDrive swerve,
                      Shooter shooter,
                      Conveyor conveyor,
                      Intake intake,
                      Flap flap,
                      Hood hood) {
        addCommands(
                new CheckSolenoids(hood, flap, intake)
                        .alongWith(new RunCommand(() -> System.out.println("Checking solenoids")))
                        .andThen(new WaitCommand(1)),
                new TurnAllMotors(shooter, conveyor, swerve, intake)
                        .alongWith(new RunCommand(() -> System.out.println("Turning all motors")))
                        .andThen(new WaitCommand(1)),
                new HelpfulZeroing(swerve)
                        .alongWith(new RunCommand(() -> System.out.println("Zeroing swerve modules")))
                        .andThen(new WaitCommand(5)),
                new TurnToRandomAngles(swerve)
                        .alongWith(new RunCommand(() -> System.out.println("Oscillating swerve modules")))
                        .andThen(new WaitCommand(1))
        );
    }
}
