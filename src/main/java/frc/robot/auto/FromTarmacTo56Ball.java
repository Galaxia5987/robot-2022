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

public class FromTarmacTo56Ball extends SaarIsAutonomous {
    public FromTarmacTo56Ball(SwerveDrive swerveDrive, Shooter shooter, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "From Tarmac to (5,6) ball");
        addCommands(shoot(0.3));
        addCommands(followPathAndPickup("From Tarmac to (5,6) ball"));

        addCommands(new ParallelDeadlineGroup(
                turnToAngle(() -> Rotation2d.fromDegrees(-31.41))
        ));
        addCommands(new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.33));

        addCommands(shootAndAdjust(10));
    }
}
