package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.Intake;
import webapp.FireLog;

import java.util.function.DoubleSupplier;

public class IntakeWithPID extends CommandBase {
    private final Intake intake;
    private final DoubleSupplier velocity;

    public IntakeWithPID(Intake intake, DoubleSupplier velocity) {
        this.intake = intake;
        this.velocity = velocity;
    }

    @Override
    public void execute() {
        FireLog.log("Intake velocity", intake.getVelocity());
        FireLog.log("Setpoint", velocity.getAsDouble());
        intake.setVelocity(velocity.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.setPower(0);
    }
}
