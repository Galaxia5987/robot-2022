package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.flap.commands.FlapCommand;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.function.BooleanSupplier;

public class Outtake extends ParallelCommandGroup {

    public Outtake(Intake intake,
                   Conveyor conveyor,
                   Flap flap,
                   Shooter shooter,
                   Hood hood,
                   BooleanSupplier condition) {
        addCommands(
                new FlapCommand(flap, Flap.FlapMode.ALLOW_SHOOTING),
                new Convey(conveyor, () -> Constants.Conveyor.DEFAULT_POWER.get() * (condition.getAsBoolean() ? 1 : -1)),
                new DynamicConditionalCommand(
                        condition,
                        new Shoot(shooter, hood, Constants.Shooter.OUTTAKE_POWER, hasTarget, odomDistance),
                        new IntakeCargo(intake, () -> -Constants.Intake.DEFAULT_POWER.get())
                )
        );
    }

}
