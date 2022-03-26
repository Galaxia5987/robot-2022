package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;

import java.util.function.BooleanSupplier;

import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class OldConvey extends CommandBase {
    private final Conveyor conveyor;
    private final BooleanSupplier preFlapSupplier;
    private final Timer timer = new Timer();
    private final Timer delayTimer = new Timer();
    private boolean last = false;
    private boolean getBallToPreFlap = true;
    private boolean wait = true;

    public OldConvey(Conveyor conveyor, BooleanSupplier preFlapSupplier) {
        this.conveyor = conveyor;
        this.preFlapSupplier = preFlapSupplier;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        timer.stop();
        delayTimer.stop();
        if (preFlapSupplier.getAsBoolean()) {
            getBallToPreFlap = false;
        }
        wait = true;
        last = false;
    }

    @Override
    public void execute() {
        if (wait) {
            double setpoint = RobotContainer.hardCodedVelocity ? RobotContainer.hardCodedVelocityValue : RobotContainer.setpointVelocity;
            if (Math.abs(setpoint - RobotContainer.Suppliers.shooterVelocity.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get()) {
                wait = false;
            }
        } else {
            if (getBallToPreFlap) {
                conveyor.setPower(Constants.Conveyor.SHOOT_POWER);
            } else {
                conveyor.setPower(0);
            }

            if (preFlapSupplier.getAsBoolean()) {
                if (!last) {
                    last = true;
                    getBallToPreFlap = false;
                    timer.start();
                    timer.reset();
                    delayTimer.start();
                    delayTimer.reset();
                }
            } else {
                last = false;
                getBallToPreFlap = true;
            }

            if (timer.hasElapsed(0.3)) {
                getBallToPreFlap = true;
                timer.reset();
                timer.stop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        delayTimer.stop();
        conveyor.setPower(0);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
