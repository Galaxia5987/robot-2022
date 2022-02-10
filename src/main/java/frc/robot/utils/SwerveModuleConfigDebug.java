package frc.robot.utils;

import frc.robot.valuetuner.WebConstant;

public final class SwerveModuleConfigDebug implements SwerveModuleConfigBase {
    private final int wheel;

    // ports
    private final int driveMotorPort;
    private final int angleMotorPort;

    // inversions
    private final boolean driveMotorInverted;
    private final boolean angleMotorInverted;
    private final boolean angleMotorSensorPhase;

    // PID
    private final WebConstant angleKp;
    private final WebConstant angleKi;
    private final WebConstant angleKd;
    private final WebConstant angleKf;

    private final WebConstant j; // moment of inertia [kg * m^2]

    private final double zeroPosition; // [ticks]

    public SwerveModuleConfigDebug(int wheel, int driveMotorPort, int angleMotorPort,
                                   boolean driveMotorInverted, boolean angleMotorInverted, boolean angleMotorSensorPhase,
                                   WebConstant angleKp, WebConstant angleKi, WebConstant angleKd, WebConstant angleKf,
                                   WebConstant j, double zeroPosition) {
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
        return angleKp.get();
    }

    @Override
    public double angleKi() {
        return angleKi.get();
    }

    @Override
    public double angleKd() {
        return angleKd.get();
    }

    @Override
    public double angleKf() {
        return angleKf.get();
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
