package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.testing.DriveForwardBITS;
import frc.robot.subsystems.drivetrain.commands.testing.HelpfulZeroing;
import frc.robot.subsystems.drivetrain.commands.testing.OscillateModules;
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
                      Hood hood,
                      Command... waitFor) {

        addRequirements(shooter);
        addCommands(
                new CheckSolenoids(hood, flap, intake)
                        .raceWith(new RunCommand(() -> System.out.println("Checking solenoids")))
                        .andThen(waitFor[0]),
                new TurnAllMotors(shooter, conveyor, swerve, intake)
                        .withTimeout(5)
                        .raceWith(new RunCommand(() -> System.out.println("Turning all motors")))
                        .andThen(new InstantCommand(() -> shooter.setVelocity(0)))
                        .andThen(waitFor[1]),
                new HelpfulZeroing(swerve)
                        .raceWith(new RunCommand(() -> System.out.println("Zeroing swerve modules")))
                        .andThen(waitFor[2]),
                new DriveForwardBITS(swerve).withTimeout(4),
                new OscillateModules(swerve)
                        .withTimeout(5)
                        .raceWith(new RunCommand(() -> System.out.println("Oscillating swerve modules")))
                        .andThen(new HelpfulZeroing(swerve))
                        .andThen(waitFor[3])
        );
    }

    public RunAllBits(SwerveDrive swerve,
                      Shooter shooter,
                      Conveyor conveyor,
                      Intake intake,
                      Flap flap,
                      Hood hood) {

        this(swerve, shooter, conveyor, intake, flap, hood,
                new WaitCommand(1),
                new WaitCommand(1),
                new WaitCommand(5),
                new WaitCommand(5));
    }
}
