package frc.robot.utils;

@FunctionalInterface
public interface MultivariableFunction {

    double apply(double... inputs);
}
