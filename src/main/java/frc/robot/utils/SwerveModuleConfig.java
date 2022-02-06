package frc.robot.utils;

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
    private final double angleKp;
    private final double angleKi;
    private final double angleKd;
    private final double angleKf;

    private final double j; // moment of inertia [kg * m^2]

    private final double zeroPosition; // [ticks]

    public SwerveModuleConfig(int wheel, int driveMotorPort, int angleMotorPort,
                              boolean driveMotorInverted, boolean angleMotorInverted, boolean angleMotorSensorPhase,
                              double angleKp, double angleKi, double angleKd, double angleKf,
                              double j, double zeroPosition) {
        this.wheel = wheel;
        this.driveMotorPort = driveMotorPort;
        this.angleMotorPort = angleMotorPort;
        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;
        this.angleMotorSensorPhase = angleMotorSensorPhase;
        this.angleKp = angleKp;
        this.angleKi = angleKi;
        this.angleKd = angleKd;
        this.angleKf = angleKf;
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
    public double angleKp() {
        return angleKp;
    }

    @Override
    public double angleKi() {
        return angleKi;
    }

    @Override
    public double angleKd() {
        return angleKd;
    }

    @Override
    public double angleKf() {
        return angleKf;
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
