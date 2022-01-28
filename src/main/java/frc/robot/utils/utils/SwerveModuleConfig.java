package frc.robot.utils.utils;

public final class SwerveModuleConfig implements SwerveModuleConfigBase {
    private final int wheel;

    // ports
    private final int driveMotorPort;
    private final int angleMotorPort;

    // inversions
    private final boolean driveMotorInverted;
    private final boolean angleMotorInverted;
    private final boolean angleMotorSensorPhase;

    // PID
    private final double angle_kp;
    private final double angle_ki;
    private final double angle_kd;
    private final double angle_kf;

    private final double j; // moment of inertia [kg * m^2]

    private final double zeroPosition; // [ticks]

    public SwerveModuleConfig(int wheel, int driveMotorPort, int angleMotorPort,
                              boolean driveMotorInverted, boolean angleMotorInverted, boolean angleMotorSensorPhase,
                              double angle_kp, double angle_ki, double angle_kd, double angle_kf,
                              double j, double zeroPosition) {
        this.wheel = wheel;
        this.driveMotorPort = driveMotorPort;
        this.angleMotorPort = angleMotorPort;
        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;
        this.angleMotorSensorPhase = angleMotorSensorPhase;
        this.angle_kp = angle_kp;
        this.angle_ki = angle_ki;
        this.angle_kd = angle_kd;
        this.angle_kf = angle_kf;
        this.j = j;
        this.zeroPosition = zeroPosition;
    }

    @Override
    public int wheel() {
        return wheel;
    }

    @Override
    public int driveMotorPort() {
        return driveMotorPort;
    }

    @Override
    public int angleMotorPort() {
        return angleMotorPort;
    }

    @Override
    public boolean driveMotorInverted() {
        return driveMotorInverted;
    }

    @Override
    public boolean angleMotorInverted() {
        return angleMotorInverted;
    }

    @Override
    public boolean angleMotorSensorPhase() {
        return angleMotorSensorPhase;
    }

    @Override
    public double angle_kp() {
        return angle_kp;
    }

    @Override
    public double angle_ki() {
        return angle_ki;
    }

    @Override
    public double angle_kd() {
        return angle_kd;
    }

    @Override
    public double angle_kf() {
        return angle_kf;
    }

    @Override
    public double j() {
        return j;
    }

    @Override
    public double zeroPosition() {
        return zeroPosition;
    }

    @Override
    public boolean debug() {
        return false;
    }
}
