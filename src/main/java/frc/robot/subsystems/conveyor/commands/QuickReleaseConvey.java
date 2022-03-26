package frc.robot.subsystems.conveyor.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.conveyor.Conveyor;
import webapp.FireLog;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Shooter.SHOOTER_VELOCITY_DEADBAND;

public class QuickReleaseConvey extends CommandBase {
    private final Conveyor conveyor;
    private final Timer timer = new Timer();
    private final DoubleSupplier velocitySupplier;
    private final Timer delayTimer = new Timer();
    private boolean wait = true;

    public QuickReleaseConvey(Conveyor conveyor, DoubleSupplier velocitySupplier) {
        this.conveyor = conveyor;
        this.velocitySupplier = velocitySupplier;
        addRequirements(conveyor);
    }

    @Override
    public void initialize() {
        timer.stop();
        delayTimer.stop();
        wait = true;
    }

    @Override
    public void execute() {
        FireLog.log("Shooter velocity", velocitySupplier.getAsDouble());
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
            conveyor.setPower(Constants.Conveyor.SHOOT_POWER);
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