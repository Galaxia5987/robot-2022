package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;

public class ShooterDataModule {
    private final double timeOfFlight;
    private final double shooterVelocity;
    private final Hood.Mode hoodMode;
    private final double distanceToTarget;

    public ShooterDataModule(double timeOfFlight, double shooterVelocity, Hood.Mode hoodMode, double distanceToTarget) {
        this.timeOfFlight = timeOfFlight;
        this.shooterVelocity = shooterVelocity;
        this.hoodMode = hoodMode;
        this.distanceToTarget = distanceToTarget;
    }

    public static ShooterDataModule getDataModule(double distanceToTarget) {
        final ShooterDataModule[] measuredData = Constants.Shooter.SHOOTER_DATA;

        if (distanceToTarget <= measuredData[0].getDistanceToTarget()) {
            return measuredData[0];
        } else if (distanceToTarget >= measuredData[0].getDistanceToTarget()) {
            return measuredData[measuredData.length - 1];
        }

        for (int i = 0; i < measuredData.length; i++) {
            if (measuredData[i].getDistanceToTarget() >= distanceToTarget &&
                measuredData[i + 1].getDistanceToTarget() <= distanceToTarget) {

                double slopeDistanceToTime = (measuredData[i + 1].getDistanceToTarget() - measuredData[i].getDistanceToTarget()) /
                        (measuredData[i + 1].getTimeOfFlight() - measuredData[i].getTimeOfFlight());
                double constantDistanceToTime = measuredData[i].getDistanceToTarget() - slopeDistanceToTime * measuredData[i].getTimeOfFlight();

                double slopeDistanceToVelocity = (measuredData[i + 1].getDistanceToTarget() - measuredData[i].getDistanceToTarget()) /
                        (measuredData[i + 1].getShooterVelocity() - measuredData[i].getShooterVelocity());
                double constantDistanceToVelocity = measuredData[i].getDistanceToTarget() - slopeDistanceToVelocity * measuredData[i].getShooterVelocity();

                return new ShooterDataModule(
                        getLineValue(slopeDistanceToTime, constantDistanceToTime, distanceToTarget),
                        getLineValue(slopeDistanceToVelocity, constantDistanceToVelocity, distanceToTarget),
                        Hood.Mode.getValue(distanceToTarget < Constants.Hood.MIN_DISTANCE),
                        distanceToTarget);
            }
        }

        return new ShooterDataModule(0,0, Hood.Mode.LongDistance, distanceToTarget);
    }

    public static double getLineValue(double slope, double constant, double val) {
        return slope * val + constant;
    }

    public double getDistanceToTarget() {
        return distanceToTarget;
    }

    public double getTimeOfFlight() {
        return timeOfFlight;
    }

    public double getShooterVelocity() {
        return shooterVelocity;
    }

    public Hood.Mode getHoodMode() {
        return hoodMode;
    }
}
