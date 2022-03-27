package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

import java.util.HashMap;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class Shoot extends CommandBase {
    protected final Shooter shooter;
    protected final Hood hood;
    protected final DoubleSupplier distance;
    private final OptionalDouble power;
    private final Timer timer = new Timer();

    public Shoot(Shooter shooter, Hood hood, double power) {
        this.shooter = shooter;
        this.hood = hood;
        this.distance = () -> 8;
        this.power = OptionalDouble.of(power);
        addRequirements(shooter);
    }

    public Shoot(Shooter shooter, Hood hood, DoubleSupplier distance) {
        this.shooter = shooter;
        this.hood = hood;
        this.distance = distance;
        this.power = OptionalDouble.empty();
        addRequirements(shooter);
    }

    /**
     * Calculates the velocity setpoint according to the distance from the target.
     * Once the data from the shooter is acquired this function will be changed.
     *
     * @param distance is the distance from the target. [m]
     * @return 15. [rpm]
     */
    public static double getSetpointVelocity(double distance) {
        HashMap<Double, Double> measurements;
        if (distance < Constants.Hood.DISTANCE_FROM_TARGET_THRESHOLD) {
            measurements = Constants.Shooter.SHORT_MEASUREMENTS;
        } else {
            measurements = Constants.Shooter.LONG_MEASUREMENTS;
        }
        double prevMeasuredDistance = 0, nextMeasuredDistance = 0;
        double minPrevDifference = Double.POSITIVE_INFINITY, minNextDifference = Double.POSITIVE_INFINITY;

        for (var measuredDistance : measurements.keySet()) {
            double difference = measuredDistance - distance;
            if (difference < 0) {
                if (Math.abs(difference) < Math.abs(minPrevDifference)) {
                    minPrevDifference = difference;
                    prevMeasuredDistance = measuredDistance;
                }
            } else {
                if (Math.abs(difference) < Math.abs(minNextDifference)) {
                    minNextDifference = difference;
                    nextMeasuredDistance = measuredDistance;
                }
            }
        }
        double y1 = measurements.get(prevMeasuredDistance);
        double y2 = measurements.get(nextMeasuredDistance);
        double t = (distance - prevMeasuredDistance) / (nextMeasuredDistance - prevMeasuredDistance);
        return (1 - t) * y1 + t * y2;
    }
/*
    @Override
    public void initialize() {
        RobotContainer.ledSubsystem.setNeutralMode(false);
    }

    @Override
    public void execute() {
        System.out.println("helloooooo");
        if (power.isEmpty()) {
            if (RobotContainer.hardCodedVelocity) {
//                shooter.setVelocity(RobotContainer.hardCodedVelocityValue);
//                SmartDashboard.putString("speed_state", Math.abs(RobotContainer.cachedSetpointForShooter - shooter.getVelocity()) <= 30 ? "green" : Math.abs(RobotContainer.cachedSetpointForShooter - shooter.getVelocity()) <= 100 ? "yellow" : "red");
            } else {
                if (RobotContainer.cachedHasTarget) {
//                    shooter.setVelocity(RobotContainer.cachedSetpointForShooter);
                    SmartDashboard.putString("speed_state", Math.abs(RobotContainer.cachedSetpointForShooter - shooter.getVelocity()) <= 30 ? "green" : Math.abs(RobotContainer.cachedSetpointForShooter - shooter.getVelocity()) <= 100 ? "yellow" : "red");
                } else {
//                    shooter.setVelocity(RobotContainer.odometryCachedSetpoint);
                    SmartDashboard.putString("speed_state", Math.abs(RobotContainer.odometryCachedSetpoint - shooter.getVelocity()) <= 30 ? "green" : Math.abs(RobotContainer.odometryCachedSetpoint - shooter.getVelocity()) <= 100 ? "yellow" : "red");
                }
            }
        } else {
            shooter.setPower(power.getAsDouble());
        }
        SmartDashboard.putNumber("something_velocity", shooter.getVelocity());
        FireLog.log("Shooter velocity2", shooter.getVelocity());
        if (RobotContainer.hardCodedVelocity) {
            FireLog.log("Shooter setpoint2", RobotContainer.hardCodedVelocityValue);
        } else {
            if (RobotContainer.cachedHasTarget) {
                FireLog.log("Shooter setpoint2", RobotContainer.cachedSetpointForShooter);
            } else {
                FireLog.log("Shooter setpoint2", RobotContainer.odometryCachedSetpoint);
            }
        }
        if (RobotContainer.hardCodedVelocity) {
            if (RobotContainer.hardCodedVelocityValue != 0) {
                RobotContainer.ledSubsystem.setPercent((int) Math.round((MathUtil.clamp(shooter.getVelocity(), 0, RobotContainer.hardCodedVelocityValue) / RobotContainer.hardCodedVelocityValue) * 9));
            }
        } else {
            if (RobotContainer.cachedHasTarget) {
                if (RobotContainer.cachedSetpointForShooter != 0) {
                    RobotContainer.ledSubsystem.setPercent((int) Math.round((MathUtil.clamp(shooter.getVelocity(), 0, RobotContainer.cachedSetpointForShooter) / RobotContainer.cachedSetpointForShooter) * 9));
                }
            } else {
                if (RobotContainer.odometryCachedSetpoint != 0) {
                    RobotContainer.ledSubsystem.setPercent((int) Math.round((MathUtil.clamp(shooter.getVelocity(), 0, RobotContainer.odometryCachedSetpoint) / RobotContainer.odometryCachedSetpoint) * 9));
                }
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
        timer.stop();
        RobotContainer.ledSubsystem.setNeutralMode(true);
        RobotContainer.ledSubsystem.setPercent(0);
        SmartDashboard.putString("speed_state", "red");
    }*/
}
