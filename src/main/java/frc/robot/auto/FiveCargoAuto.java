package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p4 - Taxi from low right tarmac and pickup low cargo(10.1)");
        addCommands(
                followPathAndPickup("p4 - Taxi from low right tarmac and pickup low cargo(10.1)")
        );

        addCommands(shootAndAdjust(3));

        addCommands(
                new ParallelRaceGroup(
                        followPath("p4 - Pickup middle cargo(10.2)"),
                        pickup(1)
                )
        );

        addCommands(
                new ParallelRaceGroup(
                        followPath("p3 - Going to terminal(9.3)"),
                        pickup(10)
                )
        );

        addCommands(new ParallelRaceGroup(
                followPath("p4 - Shooting position(10.4)"),
                new RunCommand(() -> shooter.setVelocity(3300), shooter)
        ));

        addCommands(shootAndAdjust(3));

        addCommands(followPath("p4 - Pickup up cargo(10.5)"));

        addCommands(
                new ParallelRaceGroup(
                        followPath("p4 - Going to up tarmac(10.6)"),
                        new RunCommand(() -> shooter.setVelocity(3300), shooter)
                )
        );
        addCommands(shootAndAdjust(3));
    }
}