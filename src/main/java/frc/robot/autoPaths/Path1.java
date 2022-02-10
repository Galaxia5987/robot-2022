package frc.robot.autoPaths;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;
import frc.robot.subsystems.conveyor.Conveyor;
import frc.robot.subsystems.drivetrain.SwerveDrive;
import frc.robot.subsystems.drivetrain.commands.DriveSlowAcceleration;
import frc.robot.subsystems.drivetrain.commands.tuning.DriveForward;
import frc.robot.subsystems.drivetrain.commands.tuning.Rotate;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.commands.Shoot;
import frc.robot.utils.Units;

import java.io.IOException;
import java.net.URI;
import java.nio.file.*;
import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class Path1 extends SequentialCommandGroup {
    public TrajectoryConfig trajectory = new TrajectoryConfig(Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL).setReversed(false);
    public Path path = new Path(trajectory, new Pose2d(4, 4, Rotation2d.fromDegrees(30))) {

    }
    public Path1() {
    }
}

