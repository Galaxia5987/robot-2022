package frc.robot.commandgroups.bits;

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
                new CheckSolenoids(hood, flap, intake).andThen(new WaitCommand(1)),
                new TurnAllMotors(shooter, conveyor, swerve, intake).andThen(new WaitCommand(1)),
                new HelpfulZeroing(swerve).andThen(new WaitCommand(1)),
                new TurnToRandomAngles(swerve).andThen(new WaitCommand(1))
        );
    }
}
