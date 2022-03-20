package frc.robot.auto;

import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromUpUpAndShoot extends SaarIsAutonomous {

    // Taxi from up up tarmac, shoot pre-loaded cargo, park near up tarmac.(2)
    public TaxiFromUpUpAndShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p0 - Taxi from up up tarmac(2.1)");

        addCommands(followPathAndPickup("p0 - Taxi from up up tarmac(2.1)"));

        addCommands(shootAndAdjust(3));

        addCommands(followPath("p0 - Go to up tarmac(1.2.2)"));
    }
}
