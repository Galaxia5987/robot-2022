package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
        addCommands(reachVelocityByDistance(3.35));
        addCommands(followPathAndPickup("Alon 4 ball #1"));
        addCommands(new ParallelDeadlineGroup(
                turnToAngle(() -> Rotation2d.fromDegrees(79.28)),
                new Convey(conveyor, -0.25).withTimeout(0.075)
        ));
        addCommands(new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3));
        addCommands(confirmShooting().withTimeout(2));
        addCommands(reachVelocityByDistance(4.03));
        addCommands(followPathAndPickup("Alon 4 ball #2"));
        addCommands(followPath("Alon 4 ball #3"));
        addCommands(new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3));
        addCommands(confirmShooting());
    }
}
