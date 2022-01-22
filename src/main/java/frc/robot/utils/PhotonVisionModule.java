package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.SimPhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.Optional;
import java.util.OptionalDouble;

import static frc.robot.Constants.Vision.*;

public class PhotonVisionModule extends SubsystemBase {
    private final PhotonCamera camera;
    private final SimPhotonCamera simCamera;
    private final SimVisionSystem simVisionSystem;
    private final SimulateDrivetrain simulateDrivetrain;

    public PhotonVisionModule(String cameraName, Optional<SimulateDrivetrain> simulateDrivetrain) {
        this.simulateDrivetrain = simulateDrivetrain.get();
        if (Robot.isSimulation()) {
            camera = null;
            simCamera = new SimPhotonCamera("photonvision");
            simVisionSystem = new SimVisionSystem(cameraName, DIAG_FOV, Math.toDegrees(CAMERA_PITCH), CAMERA_TO_ROBOT, CAMERA_HEIGHT, LED_RANGE, CAM_RESOLUTION_WIDTH, CAM_RESOLUTION_HEIGHT, MIN_TARGET_AREA);
            simVisionSystem.addSimVisionTarget(HUB);
        } else {
            camera = new PhotonCamera(cameraName);
            simCamera = null;
            simVisionSystem = null;
        }
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
            return OptionalDouble.of(PhotonUtils.calculateDistanceToTargetMeters(Constants.Vision.CAMERA_HEIGHT, Constants.Vision.TARGET_HEIGHT_FROM_GROUND, Constants.Vision.CAMERA_PITCH, Math.toRadians(results.getBestTarget().getPitch())));
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
            double distance = PhotonUtils.calculateDistanceToTargetMeters(Constants.Vision.CAMERA_HEIGHT, Constants.Vision.TARGET_HEIGHT_FROM_GROUND, Constants.Vision.CAMERA_PITCH, Math.toRadians(results.getBestTarget().getPitch()));
            return Optional.of(PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-results.getBestTarget().getYaw())));
        }
        return Optional.empty();
    }

    /**
     * Estimates the pose of the robot.
     *
     * @return the estimated pose and the time of detection.
     */
    public Optional<VisionEstimationData> estimatePose() {
        PhotonPipelineResult res;
        if (Robot.isSimulation()) {
            res = simCamera.getLatestResult();
        } else {
            res = camera.getLatestResult();
        }
        if (res.hasTargets()) {
            double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis() / 1000.0;
            Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
            Pose2d camPose = HUB_POSE.transformBy(camToTargetTrans.inverse());
            return Optional.of(new VisionEstimationData(camPose, imageCaptureTime));
        }
        return Optional.empty();
    }

    @Override
    public void simulationPeriodic() {
        Pose2d robotPose = simulateDrivetrain.getPose();
        simVisionSystem.processFrame(robotPose);
        SmartDashboard.putNumber("pose x", robotPose.getX());
        SmartDashboard.putNumber("pose y", robotPose.getY());
    }
}