package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.conveyor.commands.OldConvey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.utils.LedSubsystem;

public class JustShoot extends SequentialCommandGroup {
    public JustShoot(Conveyor conveyor, Flap flap, Hood hood) {
//        addCommands(new WaitUntilCommand(() -> Utils.conventionalDeadband(RobotContainer.Suppliers.yawSupplier.getAsDouble(), 5) == 0 || RobotContainer.playWithoutVision || RobotContainer.hardCodedVelocity));
        addCommands(new InstantCommand(() -> RobotContainer.warmUpShooting = true));
        addCommands(new InstantCommand(() -> LedSubsystem.currentLedMode = LedSubsystem.LedMode.SHOOTING));
        addCommands(new InstantCommand(flap::allowShooting));
        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075));
        addCommands(new InstantCommand(() -> RobotContainer.shooting = true));
        addCommands(
                new InstantCommand(() -> {
                    if (RobotContainer.hardCodedVelocity) {
                        hood.setSolenoid(RobotContainer.hardCodedDistance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance);
                    } else {
                        if (RobotContainer.hasTarget.getAsBoolean() && !RobotContainer.playWithoutVision)
                            hood.setSolenoid(RobotContainer.Suppliers.distanceSupplier.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance);
                        else
                            hood.setSolenoid(RobotContainer.Suppliers.odometryDistanceSupplier.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance);
                    }
                })
        );

        addCommands(new OldConvey(conveyor, () -> !conveyor.isPreFlapBeamConnected()));
    }
}
