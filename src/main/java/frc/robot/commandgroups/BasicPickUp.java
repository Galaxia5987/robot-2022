package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeByRobotSpeed;

import java.util.function.DoubleSupplier;

public class BasicPickUp extends ParallelCommandGroup {

    public BasicPickUp(Conveyor conveyor, Intake intake, DoubleSupplier robotVelocity) {
        addCommands(
                new IntakeByRobotSpeed(intake, robotVelocity),
                new Feed(Constants.Conveyor.DEFAULT_POWER, conveyor, () -> true)
        );
    }
}
