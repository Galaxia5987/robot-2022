package frc.robot.subsystems.helicopter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.helicopter.Helicopter;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

/**
 * parameters: Helicopter the helicopter, BooleanSupplier stop, DoubleSupplier joystickOutput.
 * This command let the driver control the climb movement by moving Xbox joystick.
 */
public class JoystickHelicopter extends CommandBase {
    private final Helicopter helicopter;
    private final DoubleSupplier joystickOutput;

    public JoystickHelicopter(Helicopter helicopter, DoubleSupplier joystickOutput) {
        this.helicopter = helicopter;
        this.joystickOutput = joystickOutput;

        addRequirements(helicopter);
    }


    @Override
    public void execute() {
        double climbVelocity = Utils.deadband(joystickOutput.getAsDouble(), Constants.Helicopter.JOYSTICK_DEADBAND) * Constants.Helicopter.MAX_VELOCITY;
        helicopter.setVelocity(climbVelocity);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > Constants.Helicopter.STOP_HELICOPTER_TIMESTAMP;
    }

    @Override
    public void end(boolean interrupted) {
        helicopter.stop();
        helicopter.setStopperMode(false);
    }
}
