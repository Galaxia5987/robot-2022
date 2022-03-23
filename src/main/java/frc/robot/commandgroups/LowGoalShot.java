package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class LowGoalShot extends ParallelCommandGroup {

    public LowGoalShot(Shooter shooter, Flap flap, Hood hood) {

        addCommands(
                new InstantCommand(() -> hood.setSolenoid(Hood.Mode.LongDistance)),
                new InstantCommand(flap::allowShooting),
                new RunCommand(() -> shooter.setVelocity(Constants.Shooter.LOW_GOAL_VELOCITY))
        );
    }
}
