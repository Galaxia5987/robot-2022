package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.commandgroups.PickUpCargo;
import frc.robot.commandgroups.ShootCargo;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.testing.SimpleAdjustWithVision;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromLowLeftPickShootPickShoot extends SaarIsAutonomous {


    // Taxi from low left tarmac, pickup middle cargo, shoot, pick up low cargo, shoot, park in low tarmac.(6)
    public TaxiFromLowLeftPickShootPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule);

        addCommands(new ParallelRaceGroup(
                        followPath.apply("p2 - Taxi from low left tarmac and pickup middle cargo(6.1)"),
                        pickup.apply(10)
                )
        );

        addCommands(shootAndAdjust.apply(5));

        addCommands(new ParallelRaceGroup(
                followPath.apply("p2 - Picking up low cargo(6.2)"),
                pickup.apply(3)));

        addCommands(shootAndAdjust.apply(3));
    }
}
