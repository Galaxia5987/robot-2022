package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class FiveCargoAuto extends SaarIsAutonomous {

    /* Taxi from low right tarmac, pickup low cargo, shoot, pickup middle cargo,
     go to terminal, pickup cargo from terminal, go to shooting position, shoot, pickup up cargo,
     park near up tarmac, shoot.(10)
     */
    public FiveCargoAuto(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "FiveCargoAutoPart1");

        addCommands(followPathAndPickup("FiveCargoAutoPart1"));

        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(83.45)));

        addCommands(shootAndAdjust(1.5));

        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(143.39)));

        addCommands(followPathAndPickup("FiveCargoAutoPart2"));

        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(37.23)));

        addCommands(shootAndAdjust(1.5));

        addCommands(followPathAndPickup("FiveCargoAutoPart3"));

        addCommands(pickup(0.5));

        addCommands(followPath("FiveCargoAutoPart4"));

        addCommands(shootAndAdjust(3));
    }
}