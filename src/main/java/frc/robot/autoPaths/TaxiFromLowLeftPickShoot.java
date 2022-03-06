package frc.robot.autoPaths;

import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromLowLeftPickShoot extends SaarIsAutonomous {

    // Taxi from low left, pick up middle cargo, shoot, park between tarmacs.(3)
    public TaxiFromLowLeftPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule module) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, module, "p1 - Taxi from low left and pickup middle cargo(3.1)");

        addCommands(
                followPathAndPickup("p1 - Taxi from low left and pickup middle cargo(3.1)")
        );

        addCommands(shootAndAdjust(5));

        addCommands(followPath("p1 - Going to middle tarmac(3.2.2)"));
    }
}
