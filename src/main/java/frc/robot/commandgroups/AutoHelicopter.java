package frc.robot.commandgroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.subsystems.helicopter.commands.AdjustAngle;
import frc.robot.subsystems.helicopter.commands.MoveHelicopter;
import frc.robot.subsystems.helicopter.commands.StopHelicopter;

public class AutoHelicopter extends SequentialCommandGroup {

    public AutoHelicopter(Helicopter helicopter,
                          double desiredRad) {

        addCommands(new MoveHelicopter(helicopter, desiredRad));
    }
}
