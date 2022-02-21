package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.subsystems.helicopter.commands.AdjustAngle;
import frc.robot.subsystems.helicopter.commands.MoveHelicopter;
import frc.robot.subsystems.helicopter.commands.StopHelicopter;

public class AutoHelicopter extends SequentialCommandGroup {

    public AutoHelicopter(Helicopter helicopter,
                          double desiredRad) {

        addCommands(
                new MoveHelicopter(helicopter, Constants.Helicopter.THIRD_RUNG),
                new WaitCommand(1),
                new MoveHelicopter(helicopter, Constants.Helicopter.RUNG_SEPARATION)
        );
    }
}
