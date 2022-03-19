package frc.robot.commandgroups;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.TurnToAngle;
import frc.robot.subsystems.drivetrain.commands.TurnWhileRunning;
import frc.robot.subsystems.flap.Flap;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.Utils;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ShootAndRun extends ParallelCommandGroup {
    
    public static Translation2d calculateCurrentGoal(double distance, double yaw) {
        return new Translation2d(distance * Math.cos(Math.toRadians(yaw)), distance * Math.sin(Math.toRadians(yaw)));
    }

    public static Translation2d calculateVirtualGoal(Translation2d currentGoal, ChassisSpeeds speeds, double flightTime) {
        return currentGoal.minus(new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond).times(flightTime));
    }

    public static double getYawToVirtualGoal(Translation2d virtualGoal, double RobotAngle) {
        return Math.toDegrees(Math.atan2(virtualGoal.getY() , virtualGoal.getX()));
    }

    public static double getShootingDistance(Translation2d virtualGoal) {
        return Math.hypot(virtualGoal.getX(), virtualGoal.getY());
    }

}
