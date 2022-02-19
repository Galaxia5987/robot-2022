package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
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
    private final DigitalOutput leds = new DigitalOutput(Ports.Vision.LEDS);

    public PhotonVisionModule(String cameraName, SimulateDrivetrain simulateDrivetrain) {
        this.simulateDrivetrain = simulateDrivetrain;
        if (Robot.isSimulation()) {
            camera = null;
            simCamera = new SimPhotonCamera("photonvision");
            simVisionSystem = new SimVisionSystem(cameraName, DIAG_FOV, CAMERA_PITCH, CAMERA_TO_ROBOT, CAMERA_HEIGHT, LED_RANGE, CAM_RESOLUTION_WIDTH, CAM_RESOLUTION_HEIGHT, MIN_TARGET_AREA);
            simVisionSystem.addSimVisionTarget(SIM_TARGET_HUB);
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
        if (Robot.isSimulation()) {
            return simCamera.getLatestResult().hasTargets();
        }
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
            return OptionalDouble.of(PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT, TARGET_HEIGHT_FROM_GROUND, Math.toRadians(CAMERA_PITCH), Math.toRadians(results.getBestTarget()
                    .getPitch())));
        }
        return OptionalDouble.empty();
    }

    public OptionalDouble getYaw() {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            return OptionalDouble.of(results.getBestTarget().getYaw());
        }
        return OptionalDouble.empty();
    }

    /**
     * Estimates the camera translation relative to the target.
     *
     * @return the translation relative to the target.
     */
    public Optional<Translation2d> estimateCameraTranslationToTarget() {
        PhotonPipelineResult results;
        if (Robot.isSimulation()) {
            results = simCamera.getLatestResult();
        } else {
            results = camera.getLatestResult();
        }
        if (results.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(CAMERA_HEIGHT, TARGET_HEIGHT_FROM_GROUND, Math.toRadians(CAMERA_PITCH), Math.toRadians(results.getBestTarget()
                    .getPitch()));
            return Optional.of(PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-results.getBestTarget()
                    .getYaw())));
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

    /**
     * @param on whether to turn on the leds.
     *           Turns the leds on or off.
     */
    public void setLeds(boolean on) {
        leds.set(on);
    }

    /**
     * Toggle the leds mode.
     */
    public void toggleLeds() {
        leds.set(!leds.get());
    }

    /**
     * @return the state of the leds - either on or off.
     */
    public boolean getLedsState() {
        return leds.get();
    }

    @Override
    public void periodic() {
        System.out.println("distance= {" + (getDistance().orElse(0) + (TARGET_RADIUS)) + "}");
    }

    @Override
    public void simulationPeriodic() {
        Pose2d robotPose = simulateDrivetrain.getPose();
        simVisionSystem.processFrame(robotPose);
        Optional<Translation2d> toTarget = estimateCameraTranslationToTarget();
        if (toTarget.isPresent()) {
            SmartDashboard.putNumber("to target x", toTarget.get().getX());
            SmartDashboard.putNumber("to target y", toTarget.get().getY());
        }
        SmartDashboard.putBoolean("hasTarget", hasTargets());
        SmartDashboard.putNumber("pose x", robotPose.getX());
        SmartDashboard.putNumber("pose y", robotPose.getY());
    }
}