package frc.robot.subsystems.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.UnitModel;
import frc.robot.utils.Utils;

import static frc.robot.Constants.LOOP_PERIOD;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Ports.Shooter.*;

public class Shooter extends SubsystemBase {
    private static final Shooter INSTANCE = new Shooter();
    private final UnitModel unitModel = new UnitModel(TICKS_PER_METER);
    private final WPI_TalonFX mainMotor = new WPI_TalonFX(MAIN_MOTOR);
    private final LinearSystemLoop<N1, N1, N1> linearSystemLoop;
    private final DCMotor motor = DCMotor.getFalcon500(1);

    private Shooter() {
        mainMotor.setInverted(MAIN_INVERTED);
        mainMotor.setSensorPhase(MAIN_SENSOR_PHASE);
        linearSystemLoop = configStateSpace("Inertia");
    }

    public static Shooter getINSTANCE() {
        return INSTANCE;
    }

    private LinearSystemLoop<N1, N1, N1> configStateSpace(String stateSpaceType) {
        LinearSystem<N1, N1, N1> flywheel_plant;
        if (stateSpaceType.equals("KaKv"))
            flywheel_plant = new LinearSystem<>(A_KaKv, B_KaKv, C_KaKv, D_KaKv);
        else if (stateSpaceType.equals("Inertia"))
            flywheel_plant = LinearSystemId.createFlywheelSystem(motor, J, GEAR_RATIO);
        else
            return null;

        LinearQuadraticRegulator<N1, N1, N1> quadraticRegulator = new LinearQuadraticRegulator<>(
                flywheel_plant,
                VecBuilder.fill(MODEL_TOLERANCE),
                VecBuilder.fill(SENSOR_TOLERANCE),
                LOOP_PERIOD);

        KalmanFilter<N1, N1, N1> kalmanFilter = new KalmanFilter<>(
                Nat.N1(), Nat.N1(),
                flywheel_plant,
                Matrix.mat(Nat.N1(), Nat.N1()).fill(MODEL_TOLERANCE),
                Matrix.mat(Nat.N1(), Nat.N1()).fill(SENSOR_TOLERANCE),
                LOOP_PERIOD);

        return new LinearSystemLoop<>(
                flywheel_plant,
                quadraticRegulator,
                kalmanFilter,
                NOMINAL_VOLTAGE, LOOP_PERIOD);
    }

    public double getVelocity() {
        return unitModel.toUnits(mainMotor.getSelectedSensorVelocity());
    }

    public void setVelocity(double velocity, double timeInterval) {
        velocity = Utils.deadband(velocity, 0.1);

        linearSystemLoop.setNextR(VecBuilder.fill(velocity));
        linearSystemLoop.correct(VecBuilder.fill(getVelocity()));
        linearSystemLoop.predict(timeInterval);

        mainMotor.set(ControlMode.PercentOutput, linearSystemLoop.getU(0) / NOMINAL_VOLTAGE);
    }

    public void terminate() {
        mainMotor.set(ControlMode.PercentOutput, 0);
    }
}