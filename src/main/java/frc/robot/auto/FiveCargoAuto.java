package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
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
        addCommands(shoot3(1.8));

//        addCommands(new InstantCommand(() -> shooter.setVelocity(3530.0)));
//
//        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075));
//
//        addCommands(quickReleaseShoot(1.8)); // 1.8

        addCommands(new InstantCommand(() -> shooter.setVelocity(3530.0)));

        addCommands(followPathAndPickup("FiveCargoAutoPart1"));

        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(40.15)));

        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075));
        addCommands(quickReleaseBackShootAndAdjust(2.5)); // 1.8

        addCommands(new InstantCommand(() -> shooter.setVelocity(3530.0)));

        addCommands(followPathAndPickup("FiveCargoAutoPart2"));

        addCommands(followPathAndPickup("FiveCargoAutoPart3"));

        addCommands(new InstantCommand(swerveDrive::terminate));

        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075));
        addCommands(quickReleaseBackShootAndAdjust(5));
    }
}