package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import webapp.FireLog;

import java.util.function.DoubleSupplier;

public class WarmUpShooter extends CommandBase {
    private final Shooter shooter;
    private final SwerveDrive swerve;
    private final Hood hood;
    private final DoubleSupplier distance;

    public WarmUpShooter(Shooter shooter, SwerveDrive swerve, Hood hood) {
        this.shooter = shooter;
        this.swerve = swerve;
        this.hood = hood;
        this.distance = this::distanceToTarget;
    }

    @Override
    public void execute() {
        double distance = this.distance.getAsDouble();
        shooter.setVelocity(Shoot.getSetpointVelocity(distance, distance < Constants.Hood.MIN_DISTANCE));

        SmartDashboard.putNumber("something_velocity", shooter.getVelocity());
        FireLog.log("Shooter velocity", shooter.getVelocity());
        FireLog.log("Shooter setpoint", Shoot.getSetpointVelocity(distance, hood.isOpen()));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.terminate();
    }

    private double distanceToTarget() {
        Transform2d delta = swerve.getPose().minus(Constants.Vision.HUB_POSE);
        return Math.hypot(delta.getX(), delta.getY());
    }
}
