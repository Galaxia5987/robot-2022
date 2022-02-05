package frc.robot.commandgroups;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Feed;
import frc.robot.subsystems.conveyor.commands.SetFlapMode;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;

import java.util.Deque;
import java.util.OptionalDouble;

public class Outtake extends ParallelCommandGroup {

    public Outtake(Intake intake,
                   Conveyor conveyor,
                   Shooter shooter,
                   double conveyorPower,
                   int remainingBalls) {
        addCommands(
                new ParallelCommandGroup(
                        new SetFlapMode(conveyor, () -> Conveyor.FlapMode.getValue(conveyor.getCargoCount() <= remainingBalls)),
                        new Feed(conveyorPower, conveyor, () -> conveyor.getCargoCount() <= remainingBalls),
                        new ConditionalCommand(
                                new Shoot(shooter, () -> 8, OptionalDouble.of(Constants.Shooter.OUTTAKE_POWER)),
                                new IntakeCargo(intake, () -> true, -Constants.Intake.DEFAULT_POWER),
                                () -> conveyorPower > 0
                        )
                ).withInterrupt(() -> conveyor.getCargoCount() <= remainingBalls)
        );
    }

    /**
     * This function is designated to returning a recommended number of balls to outtake.
     *
     * @param conveyor is the conveyor subsystem.
     * @return the recommended number of balls to leave in the conveyor.
     */
    public static int getRemainingBalls(Conveyor conveyor) {
        Deque<String> queue = conveyor.getQueue();
        if(conveyor.getCargoCount() == 2) {
            if(queue.getFirst().equals(DriverStation.getAlliance().name())) {
                if(queue.getLast().equals(DriverStation.getAlliance().name())) {
                    return 2;
                } else if(queue.getLast().equals(DriverStation.Alliance.Invalid.name())) {
                    return 0;
                } else {
                    return 1;
                }
            } else {
                return 1;
            }
        } else {
            return 0;
        }
    }
}
