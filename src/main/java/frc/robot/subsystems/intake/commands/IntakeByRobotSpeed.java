package frc.robot.subsystems.intake.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Intake.DEFAULT_POWER;
import static frc.robot.Constants.Intake.POWER_TO_VELOCITY_RATIO;

/**
 * Intakes cargo according to robot speed. (robot speed is in m/s)
 */
public class IntakeByRobotSpeed extends CommandBase {
    private final Intake intake;
    private final DoubleSupplier robotVelocity;

    public IntakeByRobotSpeed(Intake intake, DoubleSupplier robotVelocity) {
        this.intake = intake;
        this.robotVelocity = robotVelocity;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.openRetractor();
    }

    /**
     * Sets intake motor to the power calculated by multiplying robot speed by power to speed ratio.
     */
    @Override
    public void execute() {
        intake.setPower(DEFAULT_POWER.get() * (1 - robotVelocity.getAsDouble()));
    }

    /**
     * Stops intake motor.
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
}
