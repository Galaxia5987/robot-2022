package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;

public class Disappear2 extends SaarIsAutonomous {
    public Disappear2(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "Disappeare #2.2");
        addCommands(
                new ParallelRaceGroup(
                        followPathAndPickup("Disappeare #2.1"),
                        new RunCommand(() -> shooter.setVelocity(Shoot.getSetpointVelocity(Math.hypot(Constants.Vision.HUB_POSE.getX() - 4.09, Constants.Vision.HUB_POSE.getY() - 4.03))))));
        addCommands(shootAndAdjust(5));
        addCommands(followPath("Disappeare #2.2"));
    }

}
