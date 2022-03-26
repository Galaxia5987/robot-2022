package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;

public class FromTarmacTo56Ball extends SaarIsAutonomous {
    public FromTarmacTo56Ball(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule, String initialPathPath) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, initialPathPath);
        addCommands(followPath("From Tarmac to (5,6) ball"));

        addCommands(
                new ParallelRaceGroup(
                        turnToAngle(() -> Rotation2d.fromDegrees(-45)),
                        new RunCommand(() -> shooter.setVelocity(Shoot.getSetpointVelocity(Math.hypot(Constants.Vision.HUB_POSE.getX() - 5.38, Constants.Vision.HUB_POSE.getY() - 5.85))))
                )
        );
        addCommands(shootAndAdjust(2));
    }
}
