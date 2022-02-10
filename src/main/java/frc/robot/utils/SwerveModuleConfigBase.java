package frc.robot.utils;

import frc.robot.valuetuner.WebConstant;

/**
 * The interface encapsulates the module's unique properties, so it will be a lot easier and prettier to use it
 * on different modules.
 * There are 2 implementations for this interface, {@link frc.robot.utils.SwerveModuleConfig} and {@link frc.robot.utils.SwerveModuleConfigDebug},
 * The first should be used on a general basis, while the other should be used when you want to tune the PID and moment of inertia constants.
 */
public interface SwerveModuleConfigBase {
    int wheel();

    int driveMotorPort();

    int angleMotorPort();

    boolean driveMotorInverted();

    boolean angleMotorInverted();

    boolean angleMotorSensorPhase();

    double angleKp();

    double angleKi();

    double angleKd();

    double angleKf();

    double j();

    double zeroPosition();

    boolean debug();

    final class Builder {
        private final int wheel;
        private int zeroPosition;
        // ports
        private int driveMotorPort;
        private int angleMotorPort;
        // inversions
        private boolean driveMotorInverted;
        private boolean angleMotorInverted;
        private boolean angleMotorSensorPhase;
        // PID
        private double angleKp;
        private double angleKi;
        private double angleKd;
        private double angleKf;
        private double j; // moment of inertia

        private boolean debug;

        public Builder(int wheel) {
            this.wheel = wheel;
        }

        public Builder configZeroPosition(int zeroPosition) {
            this.zeroPosition = zeroPosition;
            return this;
        }

        public Builder configPorts(int drive, int angle) {
            this.driveMotorPort = drive;
            this.angleMotorPort = angle;
            return this;
        }

        public Builder configInversions(boolean driveMotorInverted, boolean angleMotorInverted, boolean angleMotorSensorPhase) {
            this.driveMotorInverted = driveMotorInverted;
            this.angleMotorInverted = angleMotorInverted;
            this.angleMotorSensorPhase = angleMotorSensorPhase;

            return this;
        }

        public Builder configAnglePID(double kp, double ki, double kd, double kf) {
            this.angleKp = kp;
            this.angleKi = ki;
            this.angleKd = kd;
            this.angleKf = kf;
            return this;
        }

        public Builder configJ(double j) {
            this.j = j;
            return this;
        }

        public Builder enableDebug() {
            this.debug = true;
            return this;
        }

        public SwerveModuleConfigBase build() {
            if (debug) {
                return new SwerveModuleConfigDebug(wheel, driveMotorPort, angleMotorPort,
                        driveMotorInverted, angleMotorInverted, angleMotorSensorPhase,
                        WebConstant.of("Swerve", wheel + "_kp", angleKp), WebConstant.of("Swerve", wheel + "_ki", angleKi),
                        WebConstant.of("Swerve", wheel + "_kd", angleKd), WebConstant.of("Swerve", wheel + "_kf", angleKf),
                        WebConstant.of("Swerve", wheel + "_j", j), zeroPosition);
            }
            return new SwerveModuleConfig(wheel, driveMotorPort, angleMotorPort,
                    driveMotorInverted, angleMotorInverted, angleMotorSensorPhase,
                    angleKp, angleKi, angleKd, angleKf, j, zeroPosition);
        }
    }
}
