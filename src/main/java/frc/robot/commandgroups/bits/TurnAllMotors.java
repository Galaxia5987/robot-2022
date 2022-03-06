package frc.robot.commandgroups.bits;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.testing.TurnAllModuleMotors;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.WarmUpShooter;

public class TurnAllMotors extends ParallelCommandGroup {
    public TurnAllMotors(Shooter shooter,
                         Conveyor conveyor,
                         SwerveDrive swerve,
                         Intake intake) {
        addCommands(
                new WarmUpShooter(shooter, () -> 3),
                new Convey(conveyor, 0.5),
                new TurnAllModuleMotors(swerve),
                new IntakeCargo(intake, () -> 0.5)
        );
    }
}
