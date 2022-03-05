package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromUpUpPickShoot extends SaarIsAutonomous {

    // Taxi from up up tarmac, pickup up cargo, shoot, park near up tarmac.(5)
    public TaxiFromUpUpPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p1 - Taxi from up up to up cargo and pickup up cargo(5.1)");

        addCommands(
                followPathAndPickup("p1 - Taxi from up up to up cargo and pickup up cargo(5.1)")
        );

        addCommands(shootAndAdjust(3));

        addCommands(followPath("p1 - Going to up tarmac(5.2.2)"));
    }
}
