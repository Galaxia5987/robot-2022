package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.utils.SwerveModuleConfigBase;
import frc.robot.utils.utils.Units;
import webapp.FireLog;

/**
 * This subsystem represents a single Swerve module and is responsible for the basic operation of a module, such as,
 * rotating to a specific angle, driving at a specific rate, and other convenience features for tuning the coefficients.
 */
public class SwerveModule extends SubsystemBase {
    private final WPI_TalonFX driveMotor;
    private final WPI_TalonSRX angleMotor;
    private final UnitModel driveUnitModel;
    private final UnitModel angleUnitModel;

    private final SwerveModuleConfigBase config;
    private LinearSystemLoop<N1, N1, N1> stateSpace;
    private double currentTime, lastTime;
    private double lastJ;

    public SwerveModule(SwerveModuleConfigBase config) {
        this.config = config;
        driveMotor = new WPI_TalonFX(config.driveMotorPort());
        angleMotor = new WPI_TalonSRX(config.angleMotorPort());
        driveUnitModel = new UnitModel(Constants.SwerveDrive.DRIVE_MOTOR_TICKS_PER_METER);
        angleUnitModel = new UnitModel(Constants.SwerveDrive.ANGLE_MOTOR_TICKS_PER_RADIAN);
        stateSpace = constructVelocityLinearSystem(config.j());
        stateSpace.reset(VecBuilder.fill(Units.metersPerSecondToRps(getVelocity(), Constants.SwerveDrive.WHEEL_RADIUS)));
        lastJ = config.j();

        // configure feedback sensors
        angleMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.TALON_TIMEOUT);
        angleMotor.configFeedbackNotContinuous(false, Constants.TALON_TIMEOUT);

        angleMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        // inversions
        driveMotor.setInverted(config.driveMotorInverted());
        angleMotor.setInverted(config.angleMotorInverted());

        driveMotor.setSensorPhase(config.driveMotorSensorPhase());
        angleMotor.setSensorPhase(config.angleMotorSensorPhase());

        // Set amperage limits
        SupplyCurrentLimitConfiguration currLimitConfig = new SupplyCurrentLimitConfiguration(
                Ports.ENABLE_CURRENT_LIMIT,
                Constants.SwerveDrive.MAX_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_CURRENT,
                Constants.SwerveModule.TRIGGER_THRESHOLD_TIME
        );

        driveMotor.configSupplyCurrentLimit(currLimitConfig);

        angleMotor.configSupplyCurrentLimit(currLimitConfig);
        angleMotor.enableCurrentLimit(Ports.ENABLE_CURRENT_LIMIT);

        // set PIDF - angle motor
        configPID(config.angle_kp(), config.angle_ki(), config.angle_kd(), config.angle_kf());
        angleMotor.config_IntegralZone(0, 5);
        angleMotor.configAllowableClosedloopError(0, angleUnitModel.toTicks(Constants.SwerveDrive.ALLOWABLE_ANGLE_ERROR));

        angleMotor.configMotionAcceleration(Constants.SwerveDrive.ANGLE_MOTION_ACCELERATION);
        angleMotor.configMotionCruiseVelocity(Constants.SwerveDrive.ANGLE_CRUISE_VELOCITY);
        angleMotor.configMotionSCurveStrength(Constants.SwerveDrive.ANGLE_CURVE_STRENGTH);

        // set voltage compensation and saturation
        angleMotor.enableVoltageCompensation(Ports.ENABLE_VOLTAGE_COMPENSATION);
        angleMotor.configVoltageCompSaturation(Ports.NOMINAL_VOLTAGE);

        angleMotor.selectProfileSlot(0, 0);
        driveMotor.selectProfileSlot(1, 0);
        driveMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);

        driveMotor.configOpenloopRamp(0, Constants.TALON_TIMEOUT);
        driveMotor.configClosedloopRamp(0, Constants.TALON_TIMEOUT);
/*
        driveMotor.configNeutralDeadband(Constants.SwerveModule.DRIVE_NEUTRAL_DEADBAND);
        angleMotor.configNeutralDeadband(Constants.SwerveModule.ANGLE_NEUTRAL_DEADBAND);
*/
    }

    /**
     * Initialize the linear system to the default values in order to use the
     * state-space model controlling the velocity of the drive motor.
     *
     * @return an object that represents the model to reach the velocity at the best rate.
     */
    private LinearSystemLoop<N1, N1, N1> constructVelocityLinearSystem(double j) {
        if (j == 0) throw new RuntimeException("j must have non-zero value");
        // https://file.tavsys.net/control/controls-engineering-in-frc.pdf Page 76
        LinearSystem<N1, N1, N1> stateSpace = LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500(1),
                j, Constants.SwerveDrive.GEAR_RATIO_DRIVE_MOTOR);
        KalmanFilter<N1, N1, N1> kalman = new KalmanFilter<>(Nat.N1(), Nat.N1(), stateSpace,
                VecBuilder.fill(Constants.SwerveDrive.MODEL_TOLERANCE),
                VecBuilder.fill(Constants.SwerveDrive.ENCODER_TOLERANCE),
                Constants.LOOP_PERIOD
        );
        LinearQuadraticRegulator<N1, N1, N1> lqr = new LinearQuadraticRegulator<>(stateSpace,
                VecBuilder.fill(Constants.SwerveDrive.VELOCITY_TOLERANCE),
                VecBuilder.fill(Constants.SwerveDrive.COST_LQR),
                Constants.LOOP_PERIOD // time between loops, DON'T CHANGE
        );
        lqr.latencyCompensate(stateSpace, Constants.LOOP_PERIOD, Constants.TALON_TIMEOUT * 0.001);

        return new LinearSystemLoop<>(stateSpace, lqr, kalman, Ports.NOMINAL_VOLTAGE, Constants.LOOP_PERIOD);
    }


    /**
     * Gets the velocity of the drive motor.
     *
     * @return the velocity of the wheel. [m/s]
     */
    public double getVelocity() {
        return driveUnitModel.toVelocity(driveMotor.getSelectedSensorVelocity(0));
    }

    /**
     * Sets the velocity of the drive motor.
     *
     * @param velocity the velocity of the module. [m/s]
     */
    public void setVelocity(double velocity) {
        double timeInterval = Math.max(Constants.LOOP_PERIOD, currentTime - lastTime);
        double targetVelocity = Units.metersPerSecondToRps(velocity, Constants.SwerveDrive.WHEEL_RADIUS);
        double currentVelocity = Units.metersPerSecondToRps(getVelocity(), Constants.SwerveDrive.WHEEL_RADIUS);

        stateSpace.setNextR(VecBuilder.fill(targetVelocity)); // r = reference (setpoint)
        stateSpace.correct(VecBuilder.fill(currentVelocity));
        stateSpace.predict(timeInterval);

        // u = input, the needed input in order to come to the next state optimally
        driveMotor.setVoltage(stateSpace.getU(0));
    }

    /**
     * Gets the angle that the module is pointing.
     *
     * @return the angle of the module. [rad]
     */
    public Rotation2d getAngle() {
        return new Rotation2d(Math.IEEEremainder(angleUnitModel.toUnits(angleMotor.getSelectedSensorPosition() - config.zeroPosition()), 2 * Math.PI));
    }

    /**
     * Sets the angle of the module, with consideration of the shortest path to the target angle.
     *
     * @param angle the target angle.
     */
    public void setAngle(Rotation2d angle) {
        var currentAngle = getAngle();
        var error = angle.minus(currentAngle);

        angleMotor.set(ControlMode.MotionMagic, angleMotor.getSelectedSensorPosition() + angleUnitModel.toTicks(error.getRadians()));
    }

    /**
     * Gets the state of the module.
     *
     * @return the current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getAngle());
    }

    /**
     * Update the state of the module.
     *
     * @param state the desired state.
     */
    public void setState(SwerveModuleState state) {
        setVelocity(state.speedMetersPerSecond);
        setAngle(state.angle);
    }

    /**
     * Stops the drive motor from moving.
     */
    public void stopDriveMotor() {
        driveMotor.stopMotor();
    }

    /**
     * Stops the angle motor from moving.
     */
    public void stopAngleMotor() {
        angleMotor.stopMotor();
    }

    /**
     * Runs the motor at full power (only used for testing purposes).
     */
    public void setMaxOutput() {
        driveMotor.set(ControlMode.PercentOutput, 1);
        angleMotor.set(ControlMode.PercentOutput, 1);
    }

    /**
     * Gets the wheel number.
     *
     * @return the wheel number.
     */
    public int getWheel() {
        return config.wheel();
    }

    /**
     * Configures the pid coefficients of the angle motor.
     *
     * @param kp proportional coefficients.
     * @param ki integral coefficients.
     * @param kd derivative coefficients.
     * @param kf feedforward coefficients.
     */
    private void configPID(double kp, double ki, double kd, double kf) {
        angleMotor.config_kP(0, kp, Constants.TALON_TIMEOUT);
        angleMotor.config_kI(0, ki, Constants.TALON_TIMEOUT);
        angleMotor.config_kD(0, kd, Constants.TALON_TIMEOUT);
        angleMotor.config_kF(0, kf, Constants.TALON_TIMEOUT);
    }

    @Override
    public void periodic() {
        if (config.debug()) {
            configPID(config.angle_kp(), config.angle_ki(), config.angle_kd(), config.angle_kf());
            if (config.j() != lastJ) {
                stateSpace = constructVelocityLinearSystem(config.j());
                stateSpace.reset(VecBuilder.fill(Units.metersPerSecondToRps(getVelocity(), Constants.SwerveDrive.WHEEL_RADIUS)));
                lastJ = config.j();
            }
        }
        FireLog.log("angle " + config.wheel(), getAngle().getDegrees());
        FireLog.log("velocity " + config.wheel(), getVelocity());
        // There is no need for kalman, so we technically disable him that way
        stateSpace.getObserver().reset();
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
    }
}
