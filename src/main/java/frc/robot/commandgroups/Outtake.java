package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.conveyor.commands.FlapDefaultCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.OptionalDouble;
import java.util.function.BooleanSupplier;

public class Outtake extends ParallelCommandGroup {

    public Outtake(Intake intake,
                   Conveyor conveyor,
                   Shooter shooter,
                   BooleanSupplier condition) {
        addCommands(
                new FlapDefaultCommand(conveyor, () -> Conveyor.FlapMode.Open),
                new Convey(conveyor, () -> Constants.Conveyor.DEFAULT_POWER * (condition.getAsBoolean() ? 1 : -1)),
                new DynamicConditionalCommand(
                        condition,
                        new Shoot(shooter, () -> 8, OptionalDouble.of(Constants.Shooter.OUTTAKE_POWER)),
                        new IntakeCargo(intake, -Constants.Intake.DEFAULT_POWER)
                )
        );
    }
}
