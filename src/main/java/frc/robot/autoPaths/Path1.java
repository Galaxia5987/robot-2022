package frc.robot.autoPaths;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;


public class Path1 extends SequentialCommandGroup {
    public TrajectoryConfig trajectory = new TrajectoryConfig(Constants.Autonomous.MAX_VEL, Constants.Autonomous.MAX_ACCEL).setReversed(false);

    public Path1() {
        PathPlanner.loadPath("name", MAX_SPEED, MAX_ACCELERATION);
    }
}

