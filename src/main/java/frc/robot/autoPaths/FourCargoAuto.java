package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class FourCargoAuto extends SaarIsAutonomous {

    // Taxi from low right tarmac, pickup low cargo, shoot, pickup middle cargo,
    // go to terminal, pickup cargo, park near low tarmac, shoot.(9)
    public FourCargoAuto(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule);

        addCommands(
                new ParallelCommandGroup(
                        followPath.apply("p3 - Taxi from low right tarmac and pickup low cargo(9.1)"),
                        pickup.apply(3)
                )
        );

        addCommands(shootAndAdjust.apply(3));

        addCommands(new ParallelCommandGroup(
                        followPath.apply("p3 - Pickup middle cargo(9.2)"),
                        pickup.apply(3)
                )
        );

        addCommands(followPath.apply("p3 - Going to terminal(9.3)"));

        addCommands(pickup.apply(3));

        addCommands(followPath.apply("p3 - Going to low tarmac(9.4.1)"));

        addCommands(shootAndAdjust.apply(3));
    }
}