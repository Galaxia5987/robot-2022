package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionEstimationData {
    private final boolean hasTarget;
    private final Pose2d estimatedPose;
    private final double time;

    public VisionEstimationData(boolean hasTarget, Pose2d estimatedPose, double time) {
        this.hasTarget = hasTarget;
        this.estimatedPose = estimatedPose;
        this.time = time;
    }

    public Pose2d estimatedPose() {
        return estimatedPose;
    }

    public double time() {
        return time;
    }

    public boolean hasTarget() {
        return hasTarget;
    }
}
