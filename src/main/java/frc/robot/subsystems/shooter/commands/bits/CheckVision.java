package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import static frc.robot.Constants.Vision.*;

public class CheckVision extends SequentialCommandGroup {


    public CheckVision(Shooter shooter, Hood hood, PhotonVisionModule photonVisionModule, PhotonCamera photonCamera) {
        addCommands(
                new ParallelRaceGroup(
                        new Shoot(
                                shooter,
                                hood,
                                photonVisionModule::getDistance).withTimeout(8),

                        new RunCommand(() -> System.out.println("Current velocity for 2m: " + shooter.getVelocity())),

                        new RunCommand(() -> System.out.println("Desired velocity for 2m" + Shoot.getSetpointVelocity(2, true)))
                ));
    }


    public double testGetDistance(PhotonVisionModule photonVisionModule, PhotonCamera photonCamera, LinearFilter linearFilter) {
        var results = photonCamera.getLatestResult();
        if (results.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                    BIT_CAMERA_HEIGHT,
                    BIT_TARGET_HEIGHT_FROM_GROUND,
                    Math.toRadians(CAMERA_PITCH),
                    Math.toRadians(results.getBestTarget().getPitch())
            );

            return linearFilter.calculate(distance) + TARGET_RADIUS;
        }
        return 0;
    }
}
