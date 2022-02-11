package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;

import static frc.robot.Constants.*;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Ports.Shooter.INVERSION_TYPE;
import static frc.robot.Ports.Shooter.MOTOR;

public class Shooter extends SubsystemBase {
    private static Shooter INSTANCE;
    private final UnitModel unitModel = new UnitModel(TICKS_PER_REVOLUTION);
    private final WPI_TalonFX motor = new WPI_TalonFX(MOTOR);
    private final LinearSystemLoop<N1, N1, N1> linearSystemLoop;
    private FlywheelSim flywheelSim;
    private TalonFXSimCollection simCollection;
    private double currentTime = 0;
    private double lastTime = 0;
    private static final ShuffleboardTab tab = Shuffleboard.getTab("Velocity");
    public static NetworkTableEntry velocity =
            tab.add("Velocity", 0)
                    .getEntry();

    private Shooter() {
        configureMotor();
        linearSystemLoop = configStateSpace(true);
        if (Robot.isSimulation()) {
            simCollection = motor.getSimCollection();
        }
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

    private void configureMotor() {
        motor.configAllSettings(getConfiguration());
        motor.setInverted(INVERSION_TYPE);
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
        if (isInertiaBased) {
            flywheel_plant = LinearSystemId.createFlywheelSystem(motor, J, GEAR_RATIO);
        } else {
            flywheel_plant = LinearSystemId.identifyVelocitySystem(Kv, Ka);
        }
        if (Robot.isSimulation()) {
            flywheelSim = new FlywheelSim(
                    flywheel_plant,
                    motor,
                    GEAR_RATIO);
        }

        LinearQuadraticRegulator<N1, N1, N1> quadraticRegulator = new LinearQuadraticRegulator<>(
                flywheel_plant,
                VecBuilder.fill(VELOCITY_TOLERANCE),
                VecBuilder.fill(COST_LQR),
                LOOP_PERIOD);
        quadraticRegulator.latencyCompensate(flywheel_plant, LOOP_PERIOD, Units.millisecondsToSeconds(TALON_TIMEOUT));

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
     * @return the velocity of the motor. [rpm]
     */
    public double getVelocity() {
        if (Robot.isSimulation()) {
            return flywheelSim.getAngularVelocityRPM();
        }
        return Utils.rpsToRpm(unitModel.toVelocity(motor.getSelectedSensorVelocity()));
    }

    /**
     * Sets the velocity of the motor.
     *
     * @param velocity is the velocity setpoint. [rpm]
     */
    public void setVelocity(double velocity) {
        linearSystemLoop.setNextR(VecBuilder.fill(Units.rotationsToRadians(Utils.rpmToRps(velocity))));
        linearSystemLoop.correct(VecBuilder.fill(Units.rotationsToRadians(Utils.rpmToRps(getVelocity()))));
        linearSystemLoop.predict(currentTime - lastTime);
        if (Robot.isSimulation()) {
            SmartDashboard.putNumber("ve", velocity);
            flywheelSim.setInputVoltage(MathUtil.clamp(linearSystemLoop.getU(0), -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE));
            simCollection.setIntegratedSensorVelocity(unitModel.toTicks100ms(Utils.rpmToRps(flywheelSim.getAngularVelocityRPM())));
        } else {
            motor.setVoltage(MathUtil.clamp(linearSystemLoop.getU(0), -NOMINAL_VOLTAGE, NOMINAL_VOLTAGE));
        }
    }

    /**
     * Set the power for the shooter motor.
     * This should only be used for testing purposes.
     *
     * @param power is the power for the motor. [%]
     */
    public void setPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
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

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("velocity", flywheelSim.getAngularVelocityRPM());
        SmartDashboard.putNumber("power", simCollection.getMotorOutputLeadVoltage());
        flywheelSim.update(currentTime - lastTime);
        lastTime = currentTime;
        currentTime = Timer.getFPGATimestamp();
    }
}