package frc.robot.valuetuner;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The class is used to update the value of the web constant through the networktables.
 */
public class NetworkTableConstant implements WebConstant {
    private static final NetworkTable BASE_TABLE = NetworkTableInstance.getDefault().getTable("value-tuner");

    private final NetworkTableEntry constant;
    private final double defaultValue;

    NetworkTableConstant(String table, String key, double defaultValue) {
        this.defaultValue = defaultValue;
        constant = BASE_TABLE.getSubTable(table).getEntry(key);
    }

    /**
     * Gets the value of the constant.
     *
     * @return the value of the constant or the default value.
     */
    @Override
    public double get() {
        return constant.getDouble(defaultValue);
    }
}
