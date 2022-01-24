package frc.robot.utils.utils;

import frc.robot.valuetuner.WebConstant;

public final class SwerveModuleConfigDebug implements SwerveModuleConfigBase {
    private final int wheel;

    // ports
    private final int driveMotorPort;
    private final int angleMotorPort;

    // inversions
    private final boolean driveMotorInverted;
    private final boolean angleMotorInverted;
    private final boolean driveMotorSensorPhase;
    private final boolean angleMotorSensorPhase;

    // PID
    private final WebConstant angle_kp;
    private final WebConstant angle_ki;
    private final WebConstant angle_kd;
    private final WebConstant angle_kf;

    private final WebConstant j; // moment of inertia [kg * m^2]

    private final double zeroPosition; // [ticks]

    public SwerveModuleConfigDebug(int wheel, int driveMotorPort, int angleMotorPort,
                                   boolean driveMotorInverted, boolean angleMotorInverted,
                                   boolean driveMotorSensorPhase, boolean angleMotorSensorPhase,
                                   WebConstant angle_kp, WebConstant angle_ki, WebConstant angle_kd, WebConstant angle_kf,
                                   WebConstant j, double zeroPosition) {
        this.wheel = wheel;
        this.driveMotorPort = driveMotorPort;
        this.angleMotorPort = angleMotorPort;
        this.driveMotorInverted = driveMotorInverted;
        this.angleMotorInverted = angleMotorInverted;
        this.driveMotorSensorPhase = driveMotorSensorPhase;
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
    public boolean driveMotorSensorPhase() {
        return driveMotorSensorPhase;
    }

    @Override
    public boolean angleMotorSensorPhase() {
        return angleMotorSensorPhase;
    }

    @Override
    public double angle_kp() {
        return angle_kp.get();
    }

    @Override
    public double angle_ki() {
        return angle_ki.get();
    }

    @Override
    public double angle_kd() {
        return angle_kd.get();
    }

    @Override
    public double angle_kf() {
        return angle_kf.get();
    }

    @Override
    public double j() {
        return j.get();
    }

    @Override
    public double zeroPosition() {
        return zeroPosition;
    }

    @Override
    public boolean debug() {
        return true;
    }
}
