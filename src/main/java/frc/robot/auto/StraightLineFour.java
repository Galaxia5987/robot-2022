package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class StraightLineFour extends SaarIsAutonomous {

    public StraightLineFour(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "");

        addCommands(
                followPathAndPickup("Four- leave low left"),
                followPath("Four- first shooting position"),
                shootAndAdjust(3),
                followPath("Four- go to terminal"),
                followPathAndPickup("Four- pickup from terminal"),
                followPath("Four- second shooting position"),
                shootAndAdjust(3)
        );
    }
}
