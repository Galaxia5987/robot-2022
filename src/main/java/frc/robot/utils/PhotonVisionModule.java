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
import frc.robot.Constants;
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
    private final LinearFilter filter = LinearFilter.movingAverage(20);

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
            return filter.calculate(PhotonUtils.calculateDistanceToTargetMeters(
                            CAMERA_HEIGHT,
                            TARGET_HEIGHT_FROM_GROUND,
                            Math.toRadians(CAMERA_PITCH),
                            Math.toRadians(results.getBestTarget().getPitch())
                    )
            ) + TARGET_RADIUS;
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

    public Translation2d ahahah() {
        double distance = getDistance();
        Rotation2d robotOrientation = Robot.getAngle();
        Rotation2d relativeAngle = Rotation2d.fromDegrees(getYaw().orElse(0));
        Rotation2d perpendicular = robotOrientation.minus(relativeAngle);
        return HUB_POSE.getTranslation().minus(new Translation2d(distance * perpendicular.getCos(), distance * perpendicular.getSin()));
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

    public Transform2d poseRelativeToTarget() {
        double navxAngle = Robot.getAngle().getDegrees();
        double yawOffset = getYaw().orElse(0);
        double d = getDistance();
        double relativeAngle = navxAngle - yawOffset + 180;
        relativeAngle = (relativeAngle < 0) ? 360 + relativeAngle : relativeAngle;
        double y = d * Math.sin(Math.toRadians(relativeAngle));
        double x = d * Math.cos(Math.toRadians(relativeAngle));

        return new Transform2d(
                new Translation2d(x, y),
                new Rotation2d(x, y)
        );
    }

    public Pose2d getOdometryWithVision() {
        return HUB_POSE.plus(poseRelativeToTarget());
    }

    public Translation2d barel() {
        PhotonPipelineResult results = Robot.isSimulation() ? simCamera.getLatestResult() : camera.getLatestResult();
        if (results.hasTargets()) {
            double d = PhotonUtils.calculateDistanceToTargetMeters(
                    CAMERA_HEIGHT,
                    TARGET_HEIGHT_FROM_GROUND,
                    Math.toRadians(CAMERA_PITCH),
                    Math.toRadians(results.getBestTarget().getPitch())
            ) + TARGET_RADIUS;
            Rotation2d targetAngle = Robot.getAngle().minus(Rotation2d.fromDegrees(-results.getBestTarget().getYaw()));
            double x = d * targetAngle.getCos();
            double y = d * targetAngle.getSin();

            int quadrant = (int) Math.floor(targetAngle.getDegrees() / 90.0);
            double dX;
            double dY;
            switch (quadrant) {
                case 0:
                    dX = Constants.FIELD_WIDTH / 2 * targetAngle.getCos();
                    if (Math.signum(targetAngle.getDegrees()) == 1) {
                        dY = Constants.FIELD_LENGTH / 2 * targetAngle.getSin();
                    } else {
                        dY = -Constants.FIELD_LENGTH / 2 * targetAngle.getSin();
                    }
                    break;
                case 1:
                    dX = -Constants.FIELD_WIDTH / 2 * targetAngle.getCos();
                    dY = Constants.FIELD_LENGTH / 2 * targetAngle.getSin();
                    break;
                default:
                    dX = -Constants.FIELD_WIDTH / 2 * targetAngle.getCos();
                    dY = -Constants.FIELD_LENGTH / 2 * targetAngle.getSin();
                    break;
            }
            System.out.println("Angle: " + targetAngle.getDegrees());
            return new Translation2d(Math.abs(x) + dX, Math.abs(y) + dY);
        }
        return new Translation2d();
    }

    @Override
    public void periodic() {
//        System.out.println("Pose with vision = " + HUB_POSE.plus(poseRelativeToTarget()));
        Pose2d pos = getOdometryWithVision();
        String outputPosition = pos.getX() + ", " + pos.getY();
        SmartDashboard.putString("robot_position", outputPosition);
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