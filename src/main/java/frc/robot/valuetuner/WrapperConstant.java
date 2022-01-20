package frc.robot.valuetuner;

public class WrapperConstant implements WebConstant {
    private final double defaultValue;

    WrapperConstant(double defaultValue) {
        this.defaultValue = defaultValue;
    }

    @Override
    public double get() {
        return defaultValue;
    }
}
