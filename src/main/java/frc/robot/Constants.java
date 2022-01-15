package frc.robot;


import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;

public final class Constants {
    public static final double LOOP_PERIOD = 0.02; // loop period. [s]
    public static final double g = 9.80665;
    public static final double UPPER_TARGET_HEIGHT = 2.64;
    public static final double PEBZNER_HEIGHT = 4.8;
    public static final int TALON_TIMEOUT = 10; // Waiting period for configurations [ms].
    public static final double NOMINAL_VOLTAGE = 12; // [volts]
    public static final boolean ENABLE_VOLTAGE_COMPENSATION = true;
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static class Shooter {
        public static final double BALL_RADIUS = 0.12; // radius of the ball. [m]
        public static final double BALL_WEIGHT = 0.27; // weight of the ball. [kg]
        public static final double AREA_OF_ATTACK = Math.PI * Math.pow(BALL_RADIUS, 2);
        public static final double Cd = 0.4;
        public static final double AIR_DENSITY = 1.225;

        public static final double NOMINAL_VOLTAGE = 12.0;
        public static final double WHEEL_RADIUS = 1;
        public static final int TICKS_PER_REVOLUTION = 2048;
        public static final double TICKS_PER_METER = Math.pow(WHEEL_RADIUS, 2) * Math.PI
                / TICKS_PER_REVOLUTION; // ticks per meter of the wheel. [tick/m]

        public static final double Ka = 1;
        public static final double Kv = 1;
        public static final Matrix<N1, N1> A_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                -Kv / Ka);
        public static final Matrix<N1, N1> B_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                1 / Ka);
        public static final Matrix<N1, N1> C_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                1);
        public static final Matrix<N1, N1> D_KaKv = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(
                0);
        public static final double MODEL_TOLERANCE = 0.8;
        public static final double SENSOR_TOLERANCE = 0.2;

        public static final double J = 0.00032;
        public static final double GEAR_RATIO = 1;
    }

    public static class AngleChanger {
        public static final double ACTIVE_ANGLE = 55;
        public static final double INACTIVE_ANGLE = 70;
    }
}
