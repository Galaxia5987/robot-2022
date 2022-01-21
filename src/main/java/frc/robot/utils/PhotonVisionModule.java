package frc.robot.utils;

import  edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import java.util.Optional;
import java.util.OptionalDouble;

public class PhotonVisionModule {
    private final PhotonCamera camera;

    public PhotonVisionModule(String cameraName) {
        camera = new PhotonCamera(cameraName);
    }

    /**
     * Check whether the camera detected a target.
     *
     * @return whether we have a target.
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Gets the distance of the robot from the target.
     *
     * @return the distance of the vision module from the target. [m]
     */
    public OptionalDouble getDistance() {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            return OptionalDouble.of(PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.Vision.CAMERA_HEIGHT,
                    Constants.Vision.TARGET_HEIGHT,
                    Constants.Vision.CAMERA_PITCH,
                    Math.toRadians(results.getBestTarget().getPitch())
            ));
        }
        return OptionalDouble.empty();
    }

    /**
     * Estimates the camera translation relative to the target.
     *
     * @return the translation relative to the target.
     */
    public Optional<Translation2d> estimateCameraTranslationToTarget() {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.Vision.CAMERA_HEIGHT,
                    Constants.Vision.TARGET_HEIGHT,
                    Constants.Vision.CAMERA_PITCH,
                    Math.toRadians(results.getBestTarget().getPitch())
            );
            return Optional.of(PhotonUtils.estimateCameraToTargetTranslation(
                    distance, Rotation2d.fromDegrees(-results.getBestTarget().getYaw())));
        }
        return Optional.empty();
    }

    /**
     * Estimates the pose of the robot.
     *
     * @return the estimated pose and the time of detection.
     */
    public Optional<VisionEstimationData> estimatePose() {
        var res = camera.getLatestResult();
        if (res.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis() / 1000.0;
            Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
            Pose2d camPose = Constants.Vision.HUB_POSE.transformBy(camToTargetTrans.inverse());
            return Optional.of(new VisionEstimationData(camPose, imageCaptureTime));
        }
        return Optional.empty();
    }
}
