package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.WarmUpShooter;
import frc.robot.utils.PhotonVisionModule;

public class FourBallAuto extends SaarIsAutonomous {
    public FourBallAuto(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "Alon 4 ball #1");
        addCommands(followPathAndPickup("Alon 4 ball #1"));
        addCommands(new ParallelCommandGroup(
                addCommands(turnToAngle(() -> Rotation2d.fromDegrees(79.28)))
                addCommands(shoot(3))
                addCommands(WarmUpShooter)
        ));
        addCommands(followPathAndPickup("Alon 4 ball #2"));
        addCommands(followPath("Alon 4 ball #3"));
        addCommands(new ParallelCommandGroup(
                addCommands(turnToAngle(() -> Rotation2d.fromDegrees(40.91)))
                addCommands(shoot(2))
        ));
    }
}
