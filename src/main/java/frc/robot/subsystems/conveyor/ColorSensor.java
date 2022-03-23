package frc.robot.subsystems.conveyor;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utils.DeadbandProximity;

import static frc.robot.Constants.Conveyor.*;

public class ColorSensor {
    private final ColorSensorV3 sensor;
    private final ColorMatch colorMatch = new ColorMatch();
    private DriverStation.Alliance lastSeenColor = DriverStation.Alliance.Invalid;
    private DriverStation.Alliance currentColor = DriverStation.Alliance.Invalid;
    private final DeadbandProximity proximity;

    public ColorSensor(I2C.Port colorSensorPort) {
        this.sensor = new ColorSensorV3(colorSensorPort);
        proximity = new DeadbandProximity(sensor::getProximity, 90, 120);

        colorMatch.addColorMatch(RED);
        colorMatch.addColorMatch(BLUE);
        colorMatch.addColorMatch(NONE);

    }

    /**
     * @return the color sensor value as a {@link edu.wpi.first.wpilibj.DriverStation.Alliance} enum.
     */
    public DriverStation.Alliance getColor() {
        if (!proximity.getState()) return DriverStation.Alliance.Invalid;
        ColorMatchResult result = colorMatch.matchClosestColor(sensor.getColor());
        Color resultColor = result.color;

        if (resultColor == RED) {
            return DriverStation.Alliance.Red;
        } else if (resultColor == BLUE) {
            return DriverStation.Alliance.Blue;
        } else {
            return DriverStation.Alliance.Invalid;
        }
    }

    /**
     * Returns the proximity of the color sensor from the nearest object.
     *
     * @return the proximity from the object (0 to 2047, largest when object is close).
     */
    public int getProximityValue() {
        return sensor.getProximity();
    }

    /**
     * Updates the values of the color sensor.
     * Call this function every loop, unless you're a psycho.
     */
    public void updateColorSensor() {
        proximity.update();
        lastSeenColor = currentColor;
        currentColor = getColor();
    }

    public DriverStation.Alliance getLastSeenColor() {
        return lastSeenColor;
    }

    public DriverStation.Alliance getCurrentColor() {
        return currentColor;
    }

    public boolean hasColorChanged() {
        return currentColor != lastSeenColor;
    }

    public double[] getRawColor() {
        return new double[]{sensor.getColor().red, sensor.getColor().green, sensor.getColor().blue};
    }
}
