package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.subsystems.helicopter.commands.MoveHelicopter;

public class AutoHelicopter extends SequentialCommandGroup {

    public AutoHelicopter(Helicopter helicopter, SwerveDrive swerveDrive) {

        addCommands(
                new TurnToAngle(swerveDrive, () ->
                        DriverStation.getAlliance() == DriverStation.Alliance.Blue ?
                                Rotation2d.fromDegrees(Constants.Helicopter.BLUE_RUNG_YAW) :
                                Rotation2d.fromDegrees(Constants.Helicopter.RED_RUNG_YAW)),
                new MoveHelicopter(helicopter, Constants.Helicopter.THIRD_RUNG),
                new WaitCommand(1),
                new MoveHelicopter(helicopter, Constants.Helicopter.RUNG_SEPARATION)
        );
    }
}
