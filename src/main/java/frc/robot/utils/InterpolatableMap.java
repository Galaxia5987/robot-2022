package frc.robot.utils;

import java.util.Map;
import java.util.TreeMap;

public class InterpolatableMap {
    private final TreeMap<Double, Double> values;

    public InterpolatableMap(Map<Double, Double> values) {
        this.values = new TreeMap<>(values);
    }

    public InterpolatableMap(double[][] values) {
        this.values = new TreeMap<>();
        for (var pair : values) {
            this.values.put(pair[0], pair[1]);
        }
    }

    public double getInterpolated(double key) {
        if (values.containsKey(key)) {
            return values.get(key);
        }

        var upperEntry = values.ceilingEntry(key);
        var lowerEntry = values.floorEntry(key);

        if (upperEntry == null) {
            upperEntry = lowerEntry;
            lowerEntry = values.lowerEntry(upperEntry.getKey());
        }
        if (lowerEntry == null) {
            lowerEntry = upperEntry;
            upperEntry = values.higherEntry(upperEntry.getKey());
        }

        double m = (upperEntry.getValue() - lowerEntry.getValue()) / (upperEntry.getKey() - lowerEntry.getKey());
        double b = upperEntry.getKey() - m * upperEntry.getValue();
        return key * m + b;
    }
}
