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

import java.util.function.DoubleSupplier;

public class FiveCargoAuto extends SaarIsAutonomous {

    /* Taxi from low right tarmac, pickup low cargo, shoot, pickup middle cargo,
     go to terminal, pickup cargo from terminal, go to shooting position, shoot, pickup up cargo,
     park near up tarmac, shoot.(10)
     */
    public FiveCargoAuto(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule);
        addCommands(
                new ParallelCommandGroup(
                        followPath.apply("p4 - Taxi from low right tarmac and pickup low cargo(10.1)"),
                        pickup.apply(3)
                )
        );

        addCommands(shootAndAdjust.apply(3));

        addCommands(
                new ParallelCommandGroup(
                        followPath.apply("p4 - Pickup middle cargo(10.2)"),
                        pickup.apply(3)
                )
        );

        addCommands(
                new ParallelCommandGroup(
                        followPath.apply("p3 - Going to terminal(9.3)"),
                        pickup.apply(3)
                )
        );

        addCommands(followPath.apply("p4 - Shooting position(10.4)"));

        addCommands(shootAndAdjust.apply(3));

        addCommands(followPath.apply("p4 - Pickup up cargo(10.5)"));

        addCommands(followPath.apply("p4 - Going to up tarmac(10.6)"));

        addCommands(shootAndAdjust.apply(3));
    }
}