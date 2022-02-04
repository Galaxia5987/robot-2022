package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeByRobotSpeed;

public class FeedWithIntake extends ParallelCommandGroup {

    public FeedWithIntake(Conveyor conveyor, Intake intake, double conveyorPower) {
        addCommands(
                new Feed(conveyorPower, conveyor),
                new ConditionalCommand(
                        new IntakeByRobotSpeed(intake, () -> 1),
                        null,
                        () -> conveyor.getCargoCount() < 2)
        );
    }
}
