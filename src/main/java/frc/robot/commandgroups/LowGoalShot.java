package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;

public class LowGoalShot extends ParallelCommandGroup {

    public LowGoalShot(Shooter shooter, Conveyor conveyor, Flap flap, Hood hood) {

        addCommands(
                new InstantCommand(() -> hood.setSolenoid(Hood.Mode.ShortDistance)),
                new InstantCommand(flap::allowShooting),
                new RunCommand(() -> shooter.setVelocity(Constants.Shooter.LOW_GOAL_VELOCITY))
        );
    }
}
