package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAndRun extends ParallelCommandGroup {
    private Shooter shooter;
    private SwerveDrive swerveDrive;
    private Hood hood;
    private Conveyor conveyor;
    private Flap flap;
    private double yawtag;

    public ShootAndRun(Shooter shooter, SwerveDrive swerveDrive, Hood hood, Conveyor conveyor, Flap flap) {


    }



    private Translation2d calculateCurrentGoal(double distance, double yaw) {
        return new Translation2d(distance * Math.cos(yaw), distance * Math.sin(yaw));
    }

    private Translation2d calculateVirtualGoal(Translation2d currentGoal, ChassisSpeeds speeds, double flightTime) {
        return currentGoal.minus(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).times(flightTime));
    }

    private double getYawToVirtualGoal(Translation2d virtualGoal) {

        return Math.atan(virtualGoal.getY()/virtualGoal.getX());
    }
    private double getShootingDistance(Translation2d virtualGoal) {
        return Math.hypot(virtualGoal.getX(), virtualGoal.getY());
    }

}
