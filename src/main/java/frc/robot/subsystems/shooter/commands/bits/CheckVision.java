package frc.robot.subsystems.shooter.commands.bits;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.PhotonVisionModule;

public class CheckVision extends SequentialCommandGroup {


    public CheckVision(Shooter shooter, Hood hood, PhotonVisionModule photonVisionModule) {
        addCommands(
                new ParallelRaceGroup(
                        new InstantCommand(() -> photonVisionModule.setLeds(false)),
                        new WaitCommand(0.5),

                        new Shoot(
                                shooter,
                                hood,
                                photonVisionModule.getDistance(Constants.Vision.BIT_CAMERA_HEIGHT, Constants.Vision.BIT_TARGET_HEIGHT_FROM_GROUND)).withTimeout(8),

                        new WaitCommand(10),
                        new RunCommand(() -> System.out.println("Current velocity for " + photonVisionModule.getDistance(Constants.Vision.BIT_CAMERA_HEIGHT, Constants.Vision.BIT_TARGET_HEIGHT_FROM_GROUND) + ": " + shooter.getVelocity())),
                        new RunCommand(() -> System.out.println("Desired velocity for " + photonVisionModule.getDistance(Constants.Vision.BIT_CAMERA_HEIGHT, Constants.Vision.BIT_TARGET_HEIGHT_FROM_GROUND) + ": " + Shoot.getSetpointVelocity(2, hood.isOpen())))
                ));
    }
}
