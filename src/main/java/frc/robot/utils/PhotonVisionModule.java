package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;
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
    private final LinearFilter filter = LinearFilter.movingAverage(10);
    private final Timer timer = new Timer();
    private boolean startedLeds = false;

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
    public double getDistance() {
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT,
                    TARGET_HEIGHT_FROM_GROUND,
                    Math.toRadians(CAMERA_PITCH),
                    Math.toRadians(results.getBestTarget().getPitch())
            );

            if (startedLeds) {
                if (!timer.hasElapsed(0.2)) {
                    filter.calculate(distance);
                    return distance + TARGET_RADIUS;
                } else {
                    startedLeds = false;
                    timer.stop();
                    timer.reset();
                }
            }

            return filter.calculate(distance) + TARGET_RADIUS;
        }
        return 0;
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
        PhotonPipelineResult results = Robot.isSimulation() ? simCamera.getLatestResult() : camera.getLatestResult();
        if (results.hasTargets()) {
            double distance = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT,
                    TARGET_HEIGHT_FROM_GROUND,
                    Math.toRadians(CAMERA_PITCH),
                    Math.toRadians(results.getBestTarget().getPitch())
            );
            return Optional.of(PhotonUtils.estimateCameraToTargetTranslation(distance, Rotation2d.fromDegrees(-results.getBestTarget().getYaw())));
        }
        return Optional.empty();
    }

    /**
     * @param off whether to turn on the leds.
     *            Turns the leds on or off.
     */
    public void setLeds(boolean off) {
        leds.set(off);
        if (!off) {
            timer.reset();
            timer.start();
        }
        startedLeds = !off;
    }

    /**
     * Toggle the leds mode.
     */
    public void toggleLeds() {
        setLeds(!getLedsState());
    }

    /**
     * @return the state of the leds - either on or off.
     */
    public boolean getLedsState() {
        return leds.get();
    }

    public VisionEstimationData estimatePose() {
        double navxAngle = Robot.getAngle().getDegrees();
        var results = camera.getLatestResult();
        if (results.hasTargets()) {
            double yawOffset = results.getBestTarget().getYaw();
            double d = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT,
                    TARGET_HEIGHT_FROM_GROUND,
                    Math.toRadians(CAMERA_PITCH),
                    Math.toRadians(results.getBestTarget().getPitch())
            );

            double relativeAngle = navxAngle - yawOffset + 180;
            relativeAngle = (relativeAngle < 0) ? 360 + relativeAngle : relativeAngle;
            double y = d * Math.sin(Math.toRadians(relativeAngle));
            double x = d * Math.cos(Math.toRadians(relativeAngle));

            double imageCaptureTime = Timer.getFPGATimestamp() - results.getLatencyMillis() / 1000.0;
            return new VisionEstimationData(true, HUB_POSE.plus(new Transform2d(
                    new Translation2d(x, y),
                    new Rotation2d(x, y)
            )), imageCaptureTime);
        }
        return new VisionEstimationData(false, null, 0);
    }

    @Override
    public void periodic() {
//        System.out.println("Distance: " + getDistance());
//        System.out.println("Pose with vision = " + HUB_POSE.plus(poseRelativeToTarget()));


        SmartDashboard.putString("visible_state", camera.getLatestResult().hasTargets() ? "green" : "red");
        double yaw = getYaw().orElse(100);
        SmartDashboard.putString("aim_state", Math.abs(yaw) <= 5 ? "green" : Math.abs(yaw) <= 13 ? "yellow" : "red");
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