package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Ports.Shooter.*;

public class Shooter extends SubsystemBase {
    private static Shooter INSTANCE;
    private final UnitModel unitModel = new UnitModel(TICKS_PER_REVOLUTION);
    private final WPI_TalonFX motor = new WPI_TalonFX(MOTOR);
    private final LinearSystemLoop<N1, N1, N1> linearSystemLoop;
    private double currentTime = 0;
    private double lastTime = 0;

    private Shooter() {
        motor.setInverted(IS_INVERTED);
        motor.setSensorPhase(IS_SENSOR_IN_PHASE);
        motor.configNeutralDeadband(NEUTRAL_DEADBAND, Constants.TALON_TIMEOUT);
        linearSystemLoop = configStateSpace(true);
    }

    /**
     * Gets the single instance of the shooter.
     *
     * @return shooter instance.
     */
    public static Shooter getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new Shooter();
        }
        return INSTANCE;
    }

    /**
     * State space configuration function. Note that there are 2 different configurations.
     *
     * @param isInertiaBased is the configuration for the state space.
     *                       If the value is true, the state space is based off of an inertia model.
     *                       Otherwise, the state space is based off of a voltage equation model.
     * @return the linear system loop (based on the type).
     */
    private LinearSystemLoop<N1, N1, N1> configStateSpace(boolean isInertiaBased) {
        final DCMotor motor = DCMotor.getFalcon500(1);

        LinearSystem<N1, N1, N1> flywheel_plant;
        if (!isInertiaBased)
            flywheel_plant = LinearSystemId.identifyVelocitySystem(Kv, Ka);
        else
            flywheel_plant = LinearSystemId.createFlywheelSystem(motor, J, GEAR_RATIO);

        LinearQuadraticRegulator<N1, N1, N1> quadraticRegulator = new LinearQuadraticRegulator<>(
                flywheel_plant,
                VecBuilder.fill(MODEL_TOLERANCE),
                VecBuilder.fill(SENSOR_TOLERANCE),
                LOOP_PERIOD);
        quadraticRegulator.latencyCompensate(flywheel_plant, LOOP_PERIOD, TALON_TIMEOUT / 1000.0);

        KalmanFilter<N1, N1, N1> kalmanFilter = new KalmanFilter<>(
                Nat.N1(), Nat.N1(),
                flywheel_plant,
                VecBuilder.fill(MODEL_TOLERANCE),
                VecBuilder.fill(SENSOR_TOLERANCE),
                LOOP_PERIOD);

        return new LinearSystemLoop<>(
                flywheel_plant,
                quadraticRegulator,
                kalmanFilter,
                NOMINAL_VOLTAGE, LOOP_PERIOD);
    }

    /**
     * Gets the velocity of the motor.
     *
     * @return the velocity of the motor. [rps]
     */
    public double getVelocity() {
        return unitModel.toVelocity(motor.getSelectedSensorVelocity());
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param velocity is the velocity setpoint. [rps]
     */
    public void setVelocity(double velocity) {
        linearSystemLoop.setNextR(VecBuilder.fill(velocity));
        linearSystemLoop.correct(VecBuilder.fill(getVelocity()));
        linearSystemLoop.predict(currentTime - lastTime);

        motor.setVoltage(Utils.clamp(linearSystemLoop.getU(0), -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE));
    }

    public void setPower(double output) {
        motor.set(ControlMode.PercentOutput, output);
    }

    /**
     * Calculates the velocity setpoint according to the distance from the target.
     *
     * @param distance is the distance from the target. [m]
     * @return 15. [rps]
     */
    public double getSetpointVelocity(double distance) {
        return 15 * distance;
    }

    /**
     * Terminates the movement of the wheel.
     */
    public void terminate() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
    }
}