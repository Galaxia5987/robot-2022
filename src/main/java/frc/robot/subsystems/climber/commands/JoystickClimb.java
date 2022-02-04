package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;

/**
 * parameters: Climber the climber, BooleanSupplier stop, DoubleSupplier joystickOutput.
 * This command let the driver control the climb movement by moving Xbox joystick.
 */
public class JoystickClimb extends CommandBase {
    private final Climber climber;
    private final DoubleSupplier joystickOutput;

    public JoystickClimb(Climber climber, DoubleSupplier joystickOutput) {
        this.climber = climber;
        this.joystickOutput = joystickOutput;

        addRequirements(climber);
    }


    @Override
    public void execute() {
        double climbVelocity = Utils.deadband(joystickOutput.getAsDouble(), Constants.Climber.JOYSTICK_DEADBAND) * Constants.Climber.MAX_VELOCITY;
        climber.setVelocity(climbVelocity);
    }

    @Override
    public boolean isFinished() {
        return Timer.getFPGATimestamp() > Constants.Climber.STOP_CLIMBER_TIMESTAMP;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
        climber.setStopperMode(false);
    }
}
