package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionEstimationData {
    private final Pose2d estimatedPose;
    private final double time;

    public VisionEstimationData(Pose2d estimatedPose, double time) {
        this.estimatedPose = estimatedPose;
        this.time = time;
    }

    public Pose2d estimatedPose() {
        return estimatedPose;
    }

    public double time() {
        return time;
    }
}
