package frc.robot;


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
        public static final double BALL_RADIUS = 0.12; // radius of the ball. [m]
        public static final double BALL_WEIGHT = 0.27; // weight of the ball. [kg]
        public static final double AREA_OF_ATTACK = Math.PI * Math.pow(BALL_RADIUS, 2); // Area that air resistance hits the ball in. [m^2]
        public static final double Cd = 0.4; // Drag coefficient of the ball.
        public static final double AIR_DENSITY = 1.225; // Average density of air. [kg/m^3]

        public static final int TICKS_PER_REVOLUTION = 2048; // Ticks per revolution of the shooter motor. [tick/c]

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
        public static final double MODEL_TOLERANCE = 0.8; // Model tolerance for state space.
        public static final double SENSOR_TOLERANCE = 0.2; // Sensor tolerance for state space.

        public static final double J = 0.00032; // Moment of inertia for state space. [kg*m^2]
        public static final double GEAR_RATIO = 1; // Gear ratio for encoder (placeholder).
    }
}
