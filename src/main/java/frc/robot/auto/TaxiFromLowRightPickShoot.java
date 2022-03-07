package frc.robot.auto;

import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromLowRightPickShoot extends SaarIsAutonomous {

    // Taxi from low right, pick up low cargo, shoot, go near low tarmac.(4)
    public TaxiFromLowRightPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p1 - Taxi from low right and pickup low cargo(4.1)");

        addCommands(
                followPathAndPickup("p1 - Taxi from low right and pickup low cargo(4.1)")
        );

        addCommands(shootAndAdjust(3));

        addCommands(followPath("p1 - Going to low tarmac(4.2.1)"));
    }
}

