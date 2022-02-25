package frc.robot.subsystems.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drivetrain.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveDrive.VELOCITY_MULTIPLIER;

public class DriveAndAdjustWithVision extends HolonomicDrive {
    private final PIDController pidController = new PIDController(Constants.SwerveDrive.HEADING_KP, 0, 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
        setTolerance(Constants.SwerveDrive.ALLOWABLE_HEADING_ERROR);
    }};

    private final PIDController adjustController = new PIDController(Constants.SwerveDrive.ADJUST_CONTROLLER_KP, Constants.SwerveDrive.ADJUST_CONTROLLER_KI.get(), 0) {{
        enableContinuousInput(-Math.PI, Math.PI);
        setTolerance(Constants.SwerveDrive.ALLOWABLE_HEADING_ERROR);
    }};
    private final Timer driftingTimer = new Timer();
    private final Timer sampleYawTimer = new Timer();
    private final DoubleSupplier yawSupplier;
    private final BooleanSupplier condition;
    private final DoubleSupplier distanceSupplier;
    private boolean newSetpoint = false;
    private Rotation2d setpoint;
    private boolean wait = false;
    private double current = 0;
    private Rotation2d target;

    public DriveAndAdjustWithVision(SwerveDrive swerveDrive, DoubleSupplier forwardSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, DoubleSupplier yawSupplier, BooleanSupplier condition, DoubleSupplier distanceSupplier) {
        super(swerveDrive, forwardSupplier, strafeSupplier, rotationSupplier);
        this.yawSupplier = yawSupplier;
        this.condition = condition;
        this.distanceSupplier = distanceSupplier;
        target = Robot.getAngle();
    }

    @Override
    public void initialize() {
        driftingTimer.reset();
        driftingTimer.start();
        sampleYawTimer.reset();
        sampleYawTimer.start();
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = calculateVelocities();
        double forward = speeds.vxMetersPerSecond;
        double strafe = speeds.vyMetersPerSecond;
        double magnitude = Math.hypot(forward, strafe);
        double alpha = Math.atan2(strafe, forward);
        if (magnitude == 0) current = 0;
        current += magnitude / 20;
        if (current > magnitude) current = magnitude;
        forward = Math.cos(alpha) * current;
        strafe = Math.sin(alpha) * current;
        double rotation = speeds.omegaRadiansPerSecond;

        if (rotation != 0) {
            setpoint = Robot.getAngle();
            newSetpoint = true;
        }

        // if there is no reason to drive
        if (magnitude == 0 && rotation == 0 && !condition.getAsBoolean()) {
            swerveDrive.terminate();
            wait = true;

            // if there is no rotation after there was rotation, start the timer.
            if (newSetpoint) {
                newSetpoint = false;
                driftingTimer.reset();
            }
            // if the time hasn't passed since there was no rotation after there was rotation, keep resetting the setpoint.
            if (!driftingTimer.hasElapsed(Constants.SwerveDrive.DRIFTING_PERIOD)) {
                setpoint = Robot.getAngle();
            }

        } else {

            // if you want to adjust to the target
            if (condition.getAsBoolean()) {
                if (yawSupplier.getAsDouble() == -100) {
                    rotation = Constants.SwerveDrive.ROTATION_MULTIPLIER / 2;
                } else {
                    if (sampleYawTimer.hasElapsed(Constants.SwerveDrive.SAMPLE_YAW_PERIOD)) {
                        Rotation2d offset = new Rotation2d(Math.atan2(-Math.signum(yawSupplier.getAsDouble()) * Constants.Shooter.CARGO_OFFSET, distanceSupplier.getAsDouble()));
                        SmartDashboard.putNumber("offset", offset.getDegrees());
                        target = Robot.getAngle().minus(Rotation2d.fromDegrees(yawSupplier.getAsDouble()).plus(offset));
                        sampleYawTimer.reset();
                    }
                    rotation = adjustController.calculate(Robot.getAngle().getRadians(), target.getRadians());
                }
            }

            // if swerveDrive angles were reached don't wait
            if (swerveDrive.haveModulesReachedAngles(forward, strafe, rotation)) {
                wait = false;
            }

            // if you want acceleration from zero speed, and angles weren't reached
            if (wait) {
                swerveDrive.errorRelativeHolonomicDrive(Math.cos(alpha) * magnitude, Math.sin(alpha) * magnitude, rotation);
                setpoint = Robot.getAngle();
                newSetpoint = true;

            } else {
                // if there is no rotation
                if (rotation == 0) {
                    // if there is no rotation after there was rotation, start the timer.
                    if (newSetpoint) {
                        newSetpoint = false;
                        driftingTimer.reset();
                    }

                    // if the time hasn't passed since there was no rotation after there was rotation, keep resetting the setpoint.
                    if (!driftingTimer.hasElapsed(Constants.SwerveDrive.DRIFTING_PERIOD)) {
                        setpoint = Robot.getAngle();
                    } else {
                        swerveDrive.defaultHolonomicDrive(forward, strafe, pidController.calculate(Robot.getAngle().getRadians(), setpoint.getRadians()));
                    }

                } else {
                    swerveDrive.defaultHolonomicDrive(forward, strafe, rotation * (1 + (current / VELOCITY_MULTIPLIER) / Constants.SwerveDrive.ROTATIONAL_ADDITION_RESTRAINT));
                    setpoint = Robot.getAngle();
                }
            }
        }
        SwerveDrive.logSpeeds(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.terminate();
    }
}