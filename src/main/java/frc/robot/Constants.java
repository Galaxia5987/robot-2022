package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;

public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // loop period. [s]
    public static final double g = 9.80665; // Gravity acceleration constant. [m/s^2]
    public static final double UPPER_TARGET_HEIGHT = 2.64; // Height of upper target. [m]
    public static final double PEBZNER_HEIGHT = 4.8; // Height of pebzner auditorium. [m]
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations. [ms]
    public static final double NOMINAL_VOLTAGE = 12.0; // Nominal voltage. [V]


    public static class Shooter {
        public static final double TICKS_PER_RADIAN = 2048 / (2 * Math.PI); // Ticks per revolution of the shooter motor. [tick]
        public static final double WHEEL_RADIUS = 0.1016;

        public static final double Ka = 1; // Acceleration state space coefficient (placeholder).
        public static final double Kv = 1; // Velocity state space coefficient (placeholder).
        public static final Matrix<N1, N1> A_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                -Kv / Ka); // Linear system A value for Ka and Kv state space.
        public static final Matrix<N1, N1> B_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                1 / Ka); // Linear system B value for Ka and Kv state space.
        public static final Matrix<N1, N1> C_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                1); // Linear system C value for Ka and Kv state space.
        public static final Matrix<N1, N1> D_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                0); // Linear system D value for Ka and Kv state space.
        public static final double MODEL_TOLERANCE = 1e-6; // Model tolerance for state space.
        public static final double SENSOR_TOLERANCE = 1e-6; // Sensor tolerance for state space.

        public static final double J = 0.000010461; // Moment of inertia for state space. [kg*m^2]
        public static final double GEAR_RATIO = 1; // Gear ratio for encoder (placeholder).
        public static final double NEUTRAL_DEADBAND = 0.1;

        public static final TalonFXConfiguration CONFIGURATION = new TalonFXConfiguration();
    }
}
