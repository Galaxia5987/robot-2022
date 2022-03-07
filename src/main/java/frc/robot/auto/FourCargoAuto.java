package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
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
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "p2 - Taxi from low right tarmac and pickup low cargo(7.1)");

        addCommands(followPathAndPickup("p2 - Taxi from low right tarmac and pickup low cargo(7.1)"));

        addCommands(shootAndAdjust(1.5));

        addCommands(new ParallelRaceGroup(
                followPath("p3 - Going to terminal(9.3)"),
                new SequentialCommandGroup(
                        new InstantCommand(intake::closeRetractor),
                        new WaitCommand(1),
                        pickup(10)
                ),
                new RunCommand(() -> shooter.setVelocity(3400), shooter)
        ));

        addCommands(followPathAndPickup("p3 - Going to middle tarmac(9.4.2)"));

        addCommands(shootAndAdjust(3));
    }
}