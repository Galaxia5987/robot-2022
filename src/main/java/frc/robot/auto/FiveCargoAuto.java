package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
        addCommands(shoot(1.5));

        addCommands(new InstantCommand(() -> shooter.setVelocity(3540.0)));

        addCommands(followPathAndPickup("FiveCargoAutoPart1"));

        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(37.23)));

        addCommands(shootAndAdjust(2.25));

//        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(143.39)));

//        addCommands(followPathAndPickup("FiveCargoAutoPart2"));
//
//        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(37.23)));
//
//        addCommands(shootAndAdjust(1));
        addCommands(new InstantCommand(() -> shooter.setVelocity(3540.0)));

        addCommands(followPathAndPickup("FiveCargoAutoPart2"));

        addCommands(pickup(0.25));

        addCommands(followPath("FiveCargoAutoPart3"));

        addCommands(shootAndAdjust(2.25));
    }
}