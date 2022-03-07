package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.intake.Intake;

import java.util.function.BooleanSupplier;

public class OneBallOuttake extends SequentialCommandGroup {
    public OneBallOuttake(Intake intake, Conveyor conveyor, BooleanSupplier hasReachedProximity) {
        addCommands(
                new RunCommand(() -> conveyor.setPower(-0.4), conveyor).withInterrupt(hasReachedProximity).withTimeout(1));
        addCommands(
                new RunCommand(() -> conveyor.setPower(0), conveyor).withTimeout(1)
        );
        addCommands(
                new ParallelRaceGroup(
                        new RunCommand(() -> intake.setPower(-Constants.Intake.DEFAULT_POWER.get()), intake),
                        new RunCommand(() -> conveyor.setPower(-0.35), conveyor).withTimeout(0.53)
                )
        );


        addCommands(
                new InstantCommand(() -> conveyor.setPower(0), conveyor)
        );

        addCommands(
                new RunCommand(() -> intake.setPower(-Constants.Intake.DEFAULT_POWER.get()), intake).withTimeout(0.2)
        );


        addCommands(
                new InstantCommand(() -> intake.setPower(0), intake)
        );


    }
}
