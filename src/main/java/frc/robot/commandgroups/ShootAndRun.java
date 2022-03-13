package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootAndRun extends ParallelCommandGroup {


    public ShootAndRun(Shooter shooter, SwerveDrive swerveDrive, Hood hood, Conveyor conveyor, Flap flap, DoubleSupplier visionDistance, DoubleSupplier visionYaw) {
        DoubleSupplier flightTime = () -> Utils.timeByDistance(visionDistance.getAsDouble());
        Supplier<Translation2d> currentGoal = () -> calculateCurrentGoal(visionDistance.getAsDouble(), visionYaw.getAsDouble());
        Supplier<Translation2d> virtualGoal = () -> calculateVirtualGoal(currentGoal.get(), swerveDrive.getChassisSpeeds(), flightTime.getAsDouble());
        addCommands(
                new ShootCargo(shooter, hood, conveyor, flap, Constants.Conveyor.DEFAULT_POWER::get, () -> getShootingDistance(virtualGoal.get())),
                new TurnToAngle(swerveDrive, () -> getYawToVirtualGoal(virtualGoal.get()))
        );

    }
    
    private Translation2d calculateCurrentGoal(double distance, double yaw) {
        return new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));
    }

    private Translation2d calculateVirtualGoal(Translation2d currentGoal, ChassisSpeeds speeds, double flightTime) {
        return currentGoal.minus(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).times(flightTime));
    }

    private double getYawToVirtualGoal(Translation2d virtualGoal) {
        return Math.atan(virtualGoal.getY() / virtualGoal.getX());
    }

    private double getShootingDistance(Translation2d virtualGoal) {
        return Math.hypot(virtualGoal.getX(), virtualGoal.getY());
    }

}
