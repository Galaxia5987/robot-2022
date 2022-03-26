package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.Convey;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;

public class JustShoot extends SequentialCommandGroup {
    public JustShoot(Conveyor conveyor, Flap flap, Hood hood) {
        addCommands(new Convey(conveyor, -0.25).withTimeout(0.075));
        addCommands(new InstantCommand(() -> RobotContainer.shooting = true));
//        if (RobotContainer.hardCodedVelocity)
//            addCommands(new InstantCommand(() -> hood.setSolenoid(RobotContainer.hardCodedDistance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
//        else {
//            if (RobotContainer.hasTarget.getAsBoolean() && !RobotContainer.playWithoutVision)
        addCommands(new InstantCommand(() -> hood.setSolenoid(RobotContainer.Suppliers.distanceSupplier.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
//            else
//                addCommands(new InstantCommand(() -> hood.setSolenoid(RobotContainer.Suppliers.odometryDistanceSupplier.getAsDouble() < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD ? Hood.Mode.ShortDistance : Hood.Mode.LongDistance)));
//        }
        addCommands(new ConveyForShooting(conveyor));
    }
}
