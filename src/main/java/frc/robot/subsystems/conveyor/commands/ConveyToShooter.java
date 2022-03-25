package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import webapp.FireLog;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class ConveyToShooter extends CommandBase {
    private final Conveyor conveyor;
    private final BooleanSupplier preFlapSupplier;
    private final Timer timer = new Timer();
    private final DoubleSupplier velocitySupplier;
    private final Timer delayTimer = new Timer();
    private boolean last = false;
    private boolean getBallToPreFlap = true;
    private boolean wait = true;
    private boolean firstBall = true;

    public ConveyToShooter(Conveyor conveyor, BooleanSupplier preFlapSupplier, DoubleSupplier velocitySupplier) {
        this.conveyor = conveyor;
        this.preFlapSupplier = preFlapSupplier;
        this.velocitySupplier = velocitySupplier;
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
        firstBall = true;
    }

    @Override
    public void execute() {
        if (RobotContainer.hardCodedVelocity) {
            FireLog.log("Shooter setpoint", RobotContainer.hardCodedVelocityValue);
        } else {
            if (RobotContainer.cachedHasTarget) {
                FireLog.log("Shooter setpoint", RobotContainer.cachedSetpointForShooter);
            } else {
                FireLog.log("Shooter setpoint", RobotContainer.odometryCachedSetpoint);
            }
        }
        if (wait) {
            FireLog.log("wait", 2000);
            if (RobotContainer.hardCodedVelocity) {
                if (Math.abs(RobotContainer.hardCodedVelocityValue - velocitySupplier.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get()) {
                    wait = false;
                }
            } else {
                if (RobotContainer.cachedHasTarget) {
                    if (Math.abs(RobotContainer.cachedSetpointForShooter - velocitySupplier.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get()) {
                        wait = false;
                    }
                } else {
                    if (Math.abs(RobotContainer.odometryCachedSetpoint - velocitySupplier.getAsDouble()) < SHOOTER_VELOCITY_DEADBAND.get()) {
                        wait = false;
                    }
                }
            }
            SmartDashboard.putString("Saar", "Mama");
        } else {
            FireLog.log("wait", 4000);
            if (firstBall) {
                conveyor.setPower(Constants.Conveyor.SHOOT_POWER);
                if (preFlapSupplier.getAsBoolean()) {
                    firstBall = false;
                }
            } else {
                SmartDashboard.putString("Saar", "Joe");
                SmartDashboard.putNumber("Saar2", timer.get());

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
                    SmartDashboard.putNumber("time", timer.get());
                    timer.reset();
                    timer.stop();
                }
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
