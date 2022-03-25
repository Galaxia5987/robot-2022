package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.ConveyToShooter;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.AdjustToTargetOnCommand;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;

public class FourBallAuto extends SaarIsAutonomous {
    public FourBallAuto(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "Alon 4 ball #1");
        addCommands(new InstantCommand(() -> RobotContainer.cachedDistanceForHood = 3.4));
        addCommands(new InstantCommand(() -> RobotContainer.cachedSetpointForShooter = Shoot.getSetpointVelocity(RobotContainer.cachedDistanceForHood)));
        addCommands(new InstantCommand(() -> shooter.setVelocity(RobotContainer.cachedSetpointForShooter)));
        addCommands(followPathAndPickup("Alon 4 ball #1"));
        addCommands(turnToAngle(() -> Rotation2d.fromDegrees(79.28)));
        new ParallelCommandGroup(
                new ConveyToShooter(conveyor, () -> !conveyor.isPreFlapBeamConnected(), shooter::getVelocity),
                new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets)
        ).withTimeout(3);
        addCommands(followPathAndPickup("Alon 4 ball #2"));
        addCommands(followPath("Alon 4 ball #3"));
        addCommands(new InstantCommand(() -> RobotContainer.cachedDistanceForHood = 4.5));
        addCommands(new InstantCommand(() -> RobotContainer.cachedSetpointForShooter = Shoot.getSetpointVelocity(RobotContainer.cachedDistanceForHood)));
        addCommands(new InstantCommand(() -> shooter.setVelocity(RobotContainer.cachedSetpointForShooter)));
        new ParallelCommandGroup(
                new ConveyToShooter(conveyor, () -> !conveyor.isPreFlapBeamConnected(), shooter::getVelocity),
                new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets)
        ).withTimeout(3);
    }
}
