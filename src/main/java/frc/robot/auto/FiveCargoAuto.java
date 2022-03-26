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
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;

public class FiveCargoAuto extends SaarIsAutonomous {

    /* Taxi from low right tarmac, pickup low cargo, shoot, pickup middle cargo,
     go to terminal, pickup cargo from terminal, go to shooting position, shoot, pickup up cargo,
     park near up tarmac, shoot.(10)
     */
    public FiveCargoAuto(Shooter shooter, SwerveDrive swerveDrive, Conveyor conveyor, Intake intake, Hood hood, Flap flap, PhotonVisionModule visionModule) {
        super(swerveDrive, shooter, conveyor, intake, hood, flap, visionModule, "FiveCargoAutoPart1");
        addCommands(reachVelocityByDistance(2.16));
        addCommands(new WaitUntilCommand(() -> Math.abs(shooter.getVelocity() - Shoot.getSetpointVelocity(2.16)) < Constants.Shooter.SHOOTER_VELOCITY_DEADBAND.get()));
        confirmShooting().withTimeout(0.6);
        addCommands(followPathAndPickup("FiveCargoAutoPart1"));
        addCommands(new ParallelDeadlineGroup(
                turnToAngle(() -> Rotation2d.fromDegrees(40.15)),
                new Convey(conveyor, -0.25).withTimeout(0.075)
        ));
        addCommands(new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3));
        addCommands(confirmShooting().withTimeout(2));
        addCommands(reachVelocityByDistance(3.89));
        addCommands(followPathAndPickup("FiveCargoAutoPart2"));
        addCommands(followPathAndPickup("FiveCargoAutoPart3"));
        addCommands(new InstantCommand(swerveDrive::terminate));
        addCommands(new AdjustToTargetOnCommand(swerveDrive, () -> visionModule.getYaw().orElse(0), visionModule::hasTargets).withTimeout(0.3));
        addCommands(confirmShooting());
    }
}