package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromUpUpPickShootTerminalShoot extends SaarIsAutonomous {

    // Taxi from up up, pickup up cargo, go to terminal, park near up tarmac, shoot.(8)
    public TaxiFromUpUpPickShootTerminalShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p2 - Taxi from up up tarmac and going to up cargo(8.1)");

        addCommands(
                followPathAndPickup("p2 - Taxi from up up tarmac and going to up cargo(8.1)")
        );

        addCommands(shootAndAdjust(3));

        addCommands(
                new ParallelRaceGroup(
                        followPath("p2 - Going to terminal(8.2)"),
                        pickup(10)
                )
        );

        addCommands(followPathAndPickup("p2 - Going to low tarmac(8.3.1)"));

        addCommands(shootAndAdjust(3));
    }

}
