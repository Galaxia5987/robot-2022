package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.utils.LedSubsystem;

public class OneButtonAdjustAndShoot extends SequentialCommandGroup {
    public OneButtonAdjustAndShoot(SwerveDrive swerve, Conveyor conveyor, Flap flap, Hood hood) {
        addCommands(new InstantCommand(flap::allowShooting));

        addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setCurrentLedMode(LedSubsystem.LedMode.ODOMETRY_ADJUST))); // LEDs

        addCommands(new ParallelCommandGroup(
                new Convey(conveyor, -0.25).withTimeout(0.075),
                //if (!RobotContainer.hasTarget.getAsBoolean())
                new OdometryAdjust(swerve)
        ).withTimeout(1));

        addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setCurrentLedMode(LedSubsystem.LedMode.VISION_ADJUST))); // LEDs

//        if (RobotContainer.hasTarget.getAsBoolean() && !RobotContainer.playWithoutVision)
        addCommands(new VisionAdjust(swerve).withTimeout(0.5));
        addCommands(new InstantCommand(() -> RobotContainer.shooting = true));
//        if (RobotContainer.hardCodedVelocity)
//            addCommands(new InstantCommand(() -> hood.setSolenoid(RobotContainer.hardCodedDistance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
//        else {
//            if (RobotContainer.hasTarget.getAsBoolean() && !RobotContainer.playWithoutVision)
        addCommands(new InstantCommand(() -> hood.setSolenoid(RobotContainer.Suppliers.distanceSupplier.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
//            else
//                addCommands(new InstantCommand(() -> hood.setSolenoid(RobotContainer.Suppliers.odometryDistanceSupplier.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
//        }

        addCommands(new InstantCommand(() -> RobotContainer.ledSubsystem.setCurrentLedMode(LedSubsystem.LedMode.SHOOTING))); // LEDs

        addCommands(new ConveyForShooting(conveyor));
    }
}
