package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import frc.robot.subsystems.intake.commands.IntakeByRobotSpeed;
import frc.robot.subsystems.intake.commands.IntakeCargo;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;

public class TaxiFromLowRightPickShootPickShoot extends SaarIsAutonomous {

    // Taxi from low right tarmac, pickup low cargo, shoot, pick up middle cargo, shoot, park near low tarmac.(7)
    public TaxiFromLowRightPickShootPickShoot(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule);

        addCommands(new ParallelRaceGroup(
                        followPath.apply("p2 - Taxi from low right tarmac and pickup low cargo(7.1)"),
                        pickup.apply(10)
                )
        );

        addCommands(shootAndAdjust.apply(3));

        addCommands(
                new ParallelRaceGroup(
                        followPath.apply("p2 - Picking up middle cargo(7.2)"),
                        pickup.apply(10),
                        new RunCommand(() -> shooter.setVelocity(Shoot.getSetpointVelocity(4, false)))
                )
        );

        addCommands(shootAndAdjust.apply(3));
    }
}
