package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

public class JoystickPowerHelicopter extends CommandBase {
    private final Helicopter helicopter;
    private final DoubleSupplier joystickOutput;

    public JoystickPowerHelicopter(Helicopter helicopter, DoubleSupplier joystickOutput) {
        this.helicopter = helicopter;
        this.joystickOutput = joystickOutput;

        addRequirements(helicopter);
    }

    @Override
    public void execute() {
        double flyFly = Utils.deadband(joystickOutput.getAsDouble(), Constants.Helicopter.JOYSTICK_DEADBAND);
        helicopter.vroomVroom(flyFly);
    }

    @Override
    public void end(boolean interrupted) {
        helicopter.stop();
        helicopter.setStopperMode(false);
    }
}


