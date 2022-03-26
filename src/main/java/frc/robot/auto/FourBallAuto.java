package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.AdjustToTargetOnCommand;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.PhotonVisionModule;

public class FourBallAuto extends SaarIsAutonomous {
    public FourBallAuto(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "Alon 4 ball #1");
        addRequirements(shooter);
        addCommands(reachVelocityByDistance(3.25));
        addCommands(new InstantCommand(intake::openRetractor));
        addCommands(new WaitCommand(0.1));
        addCommands(followPathAndPickup("Alon 4 ball #1"));
        addCommands(new ParallelDeadlineGroup(
                turnToAngle(() -> Rotation2d.fromDegrees(79.28)),
                new Convey(conveyor, -0.25).withTimeout(0.075)
        ));
        addCommands(new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3));
        addCommands(new InstantCommand((flap::allowShooting)));
        addCommands(confirmShootingSlower().withTimeout(1.7));
        addCommands(reachVelocityByDistance(4));
        addCommands(new InstantCommand((flap::blockShooter)));
        addCommands(followPathAndPickup("Alon 4 ball #2"));
        addCommands(followPathAndPickup("Alon 4 ball #3"));
        addCommands(new ParallelDeadlineGroup(
                new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3),
                new Convey(conveyor, -0.25).withTimeout(0.075)
        ));
        addCommands(new InstantCommand((flap::allowShooting)));
        addCommands(confirmShootingSlower());
    }
}
