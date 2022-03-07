package frc.robot.auto;

import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromLowRightPickShootPickShoot extends SaarIsAutonomous {

    // Taxi from low right tarmac, pickup low cargo, shoot, pick up middle cargo, shoot, park near low tarmac.(7)
    public TaxiFromLowRightPickShootPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p2 - Taxi from low right tarmac and pickup low cargo(7.1)");

        addCommands(followPathAndPickup("p2 - Taxi from low right tarmac and pickup low cargo(7.1)"));

        addCommands(shootAndAdjust(3));

        addCommands(followPathAndPickup("p2 - Picking up middle cargo(7.2)"));

        addCommands(shootAndAdjust(3));
    }
}
