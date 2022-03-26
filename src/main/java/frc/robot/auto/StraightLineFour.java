package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class StraightLineFour extends SaarIsAutonomous {

    public StraightLineFour(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "Four- leave low left");

        addCommands(
                followPathAndPickup("Four- leave low left")
        );

        addCommands(
                pickup(2)
        );

        addCommands(new ParallelRaceGroup(
                followPath("Four- first shooting position"),
                warmup(3400)
        ));

        addCommands(
                shootAndAdjust(3)
        );

        addCommands(
                followPath("Four- go to terminal")
        );

        addCommands(
                followPathAndPickup("Four- pickup from terminal")
        );

        addCommands(
                pickup(2)
        );

        addCommands(
                new ParallelRaceGroup(
                        warmup(3400),
                        followPath("Four- second shooting position")
                ));

        addCommands(
                shootAndAdjust(3)
        );

    }
}
