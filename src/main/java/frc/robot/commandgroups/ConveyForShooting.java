package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.conveyor.commands.ConveyCargo;

public class ConveyForShooting extends SequentialCommandGroup {
    public ConveyForShooting(Conveyor conveyor) {
        addCommands(new WaitUntilCommand(() -> Math.abs(RobotContainer.Suppliers.shooterVelocity.getAsDouble() - RobotContainer.setpointVelocity) < Constants.Shooter.SHOOTER_VELOCITY_DEADBAND.get()));
        addCommands(new ConveyCargo(conveyor));
    }
}
