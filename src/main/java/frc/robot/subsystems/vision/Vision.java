package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import static frc.robot.Constants.Camera.CAMERA_HEIGHT_METERS;
import static frc.robot.Constants.Camera.CAMERA_PITCH_RADIANS;
import static frc.robot.Constants.Field.HUB_POSITION;
import static frc.robot.Constants.Field.TARGET_HEIGHT_METERS;

public class Vision extends SubsystemBase {
    private final PhotonCamera photonCamera = new PhotonCamera("myCamera");

    public Vision() {

    }

    public double getTargetYaw() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (!result.hasTargets()) return 0;
        return result.getBestTarget().getYaw();
    }

    public double getTargetDistance() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (!result.hasTargets()) return 0;
        return PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
    }

    public double getTargetDistance(PhotonPipelineResult result) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                TARGET_HEIGHT_METERS,
                CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));
    }

    public Translation2d getRobotTranslation2d() {
        PhotonPipelineResult result = photonCamera.getLatestResult();
        if (!result.hasTargets()) return null;
        double distance = getTargetDistance(result);
        Rotation2d gyroAngle = Robot.getAngle();
        Rotation2d relativeAngle = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
        Rotation2d perpendicularAngle = gyroAngle.minus(relativeAngle);
        Translation2d translation2d = new Translation2d(distance * Math.cos(perpendicularAngle.getRadians()), distance * Math.sin(perpendicularAngle.getRadians()));
        return HUB_POSITION.minus(translation2d);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("VISION-YAW", getTargetYaw());
        SmartDashboard.putNumber("VISION-DISTANCE", getTargetDistance());
        Translation2d robotPose = getRobotTranslation2d();
        if (robotPose == null) robotPose = new Translation2d();
        SmartDashboard.putString("VISION-POSE", "[" + robotPose.getX() + ", " + robotPose.getY() + "]");
    }
}
