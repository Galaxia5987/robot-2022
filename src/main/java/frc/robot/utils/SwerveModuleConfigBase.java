package frc.robot.utils;

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

    double angle_kp();

    double angle_ki();

    double angle_kd();

    double angle_kf();

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
        private double angle_kp;
        private double angle_ki;
        private double angle_kd;
        private double angle_kf;
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
            this.angle_kp = kp;
            this.angle_ki = ki;
            this.angle_kd = kd;
            this.angle_kf = kf;
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
//            if (debug) {
//                return new SwerveModuleConfigDebug(wheel, driveMotorPort, angleMotorPort,
//                        driveMotorInverted, angleMotorInverted, angleMotorSensorPhase,
//                        new WebConstant("Swerve_" + wheel + "_kp", angle_kp), new WebConstant("Swerve_" + wheel + "_ki", angle_ki),
//                        new WebConstant("Swerve_" + wheel + "_kd", angle_kd), new WebConstant("Swerve_" + wheel + "_kf", angle_kf),
//                        new WebConstant("Swerve_" + wheel + "_j", j), zeroPosition);
//            }
            return new SwerveModuleConfig(wheel, driveMotorPort, angleMotorPort,
                    driveMotorInverted, angleMotorInverted, angleMotorSensorPhase,
                    angle_kp, angle_ki, angle_kd, angle_kf, j, zeroPosition);
        }
    }
}
