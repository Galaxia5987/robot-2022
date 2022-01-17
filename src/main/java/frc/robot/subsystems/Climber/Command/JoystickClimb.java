package frc.robot.subsystems.Climber.Command;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.utils.Utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * parameters: Climber climber, BooleanSupplier stop, DoubleSupplier joystickOutput.
 * this command let the driver control the climb movement by moving xbox joystick.
 */

public class JoystickClimb extends CommandBase {
    private final Climber climber;
    private final BooleanSupplier stop;
    private final DoubleSupplier joystickOutput;

    public JoystickClimb(Climber climber, BooleanSupplier stop, DoubleSupplier joystickOutput) {
        this.climber = climber;
        this.stop = stop;
        this.joystickOutput = joystickOutput;

        addRequirements(climber);
    }

    @Override
    public void execute() {
        double climbVelocity = Utils.deadband(joystickOutput.getAsDouble(), Constants.Climber.THRESHOLD) * Constants.Climber.MAX_VELOCITY;
        climber.setVelocity(climbVelocity);
    }

    @Override
    public boolean isFinished() {
        return stop.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
