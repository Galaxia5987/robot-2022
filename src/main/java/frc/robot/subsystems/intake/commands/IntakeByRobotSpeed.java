package frc.robot.subsystems.intake.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;

import java.util.function.DoubleSupplier;

/**
 * intakes balls according to robot speed.
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
        intake.setPower(Constants.Intake.BIAS + (robotVelocity.getAsDouble() * Constants.Intake.POWER_TO_VELOCITY));
    }

    /**
     * Stops intake motor.
     */
    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
}
