package frc.robot.utils;

import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Constants;

public interface SmoothedInput {
    LinearFilter joystickFilter = LinearFilter.movingAverage(
            Constants.Control.JOYSTICK_FILTER_TAP);

    default double smoothed() {
        double filteredValue = joystickFilter.calculate(getAsDouble());
        return Math.pow(Math.abs(filteredValue), Constants.Control.JOYSTICK_SMOOTHING_EXPONENT) * Math.signum(filteredValue);
    }

    double getAsDouble();
}
