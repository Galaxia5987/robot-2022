package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.AdjustToTargetOnCommand;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.LedSubsystem;
import frc.robot.utils.PhotonVisionModule;

public class FiveCargoAuto extends SaarIsAutonomous {

    /* Taxi from low right tarmac, pickup low cargo, shoot, pickup middle cargo,
     go to terminal, pickup cargo from terminal, go to shooting position, shoot, pickup up cargo,
     park near up tarmac, shoot.(10)
     */
    public FiveCargoAuto(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "FiveCargoAutoPart1");
        addRequirements(shooter);
        addCommands(reachVelocity(3350, 1));
        addCommands(new WaitUntilCommand(() -> Math.abs(shooter.getVelocity() - 3350) < Constants.Shooter.SHOOTER_VELOCITY_DEADBAND.get()));
        addCommands(new InstantCommand(flap::allowShooting));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.SHOOTING);
        addCommands(confirmShooting().withTimeout(0.6));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.STATIC);
        addCommands(reachVelocityByDistance(3.85));
        addCommands(followPathAndPickup("FiveCargoAutoPart1"));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.ODOMETRY_ADJUST);
        addCommands(new ParallelDeadlineGroup(
                turnToAngle(() -> Rotation2d.fromDegrees(40.15)),
                new Convey(conveyor, -0.25).withTimeout(0.075)
        ));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.VISION_ADJUST);
        addCommands(new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3));
        addCommands(new InstantCommand(flap::allowShooting));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.SHOOTING);
        addCommands(confirmShooting().withTimeout(1.6));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.STATIC);
        addCommands(reachVelocityByDistance(5.6));
        addCommands(followPathAndPickup("FiveCargoAutoPart2"));
        addCommands(followPathAndPickup("FiveCargoAutoPart3"));
        addCommands(new InstantCommand(swerveDrive::terminate));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.VISION_ADJUST);
        addCommands(new ParallelDeadlineGroup(
                new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3),
                new Convey(conveyor, -0.25).withTimeout(0.075)

        ));
        addCommands(new InstantCommand(flap::allowShooting));
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.SHOOTING);
        addCommands(confirmShooting());
        new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.STATIC);
    }
}