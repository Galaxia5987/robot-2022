package frc.robot.subsystems.climber.commands;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.climber.Climber;
import frc.robot.utils.Utils;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * parameters: Climber the climber, BooleanSupplier stop, DoubleSupplier joystickOutput.
 * This command let the driver control the climb movement by moving Xbox joystick.
 */

public class JoystickClimb extends CommandBase {
    private final Climber climber;
    private final BooleanSupplier isFinished;
    private final BooleanSupplier isStopper;
    private final DoubleSupplier joystickOutput;
    private boolean isReady = true;

    public JoystickClimb(Climber climber, BooleanSupplier isFinished, BooleanSupplier isStopper, DoubleSupplier joystickOutput) {
        this.climber = climber;
        this.isFinished = isFinished;
        this.isStopper = isStopper;
        this.joystickOutput = joystickOutput;

        addRequirements(climber);
    }

    @Override
    public void initialize() {
        if (Robot.isReal() && !Robot.debug)
            isReady = Timer.getFPGATimestamp() > Constants.Climber.AUTONOMY_TIME;
    }

    @Override
    public void execute() {
        double climbVelocity = Utils.deadband(joystickOutput.getAsDouble(), Constants.Climber.THRESHOLD) * Constants.Climber.MAX_VELOCITY;
        climber.setVelocity(climbVelocity);
        if (isStopper.getAsBoolean()) {
            climber.toggleStopper();
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished.getAsBoolean() || !isReady || Timer.getFPGATimestamp() > Constants.Climber.STOP_CLIMBER;
    }

    @Override
    public void end(boolean interrupted) {
        climber.stop();
    }
}
