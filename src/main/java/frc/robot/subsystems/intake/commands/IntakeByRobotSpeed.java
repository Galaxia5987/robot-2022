package frc.robot.subsystems.intake.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

import java.util.function.DoubleSupplier;

/**
 * Default intake but sucks ball according to robot speed.
 */
public class IntakeByRobotSpeed extends CommandBase {
    private final Intake intake;
    private final DoubleSupplier robotSpeedMeterPerSecond;

    public IntakeByRobotSpeed(Intake intake, DoubleSupplier speed) {
        this.intake = intake;
        this.robotSpeedMeterPerSecond = speed;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.openSolenoid();
    }

    /**
     * Sets intake motor to the power calculated by multiplying robot speed by power to speed ratio.
     */
    @Override
    public void execute() {
        intake.setPower(robotSpeedMeterPerSecond.getAsDouble() * Constants.Intake.POWER_SPEED_RATIO);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Stops intake motor.
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
}
