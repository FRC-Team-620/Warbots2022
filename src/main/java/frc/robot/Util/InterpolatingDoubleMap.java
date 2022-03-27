package frc.robot.Util;

import java.util.TreeMap;

public class InterpolatingDoubleMap extends TreeMap<Double, Double> {
    private final int maxSize;

    public InterpolatingDoubleMap(int maxSize) {
        this.maxSize = maxSize;
    }

    public InterpolatingDoubleMap() {
        this(0);
    }

    @Override
    public Double put(Double key, Double value) {
        if (this.maxSize > 0 && this.maxSize <= this.size()) { // TODO: Document this behavior, throw an error or avaoid
                                                               // it all together via a constructor.
            this.remove(this.firstKey());
        }

        return super.put(key, value);
    }

    public Double put(double key, double value) {
        return this.put(Double.valueOf(key), Double.valueOf(value));
    }

    public double getInterpolated(double key) {
        return Double.valueOf(this.getInterpolated(Double.valueOf(key)));
    }

    private Double getInterpolated(Double key) {// TODO: Document or change Behavior when key outside of the table is
                                                // used. (JavaDocs)
        Double value = this.get(key);

        if (value == null) {
            // Get upper and lower keys for interpolation
            Double lowerKey = this.floorKey(key);
            Double upperKey = this.ceilingKey(key);

            // Return the nearest data point if at tree edge
            if (lowerKey == null && upperKey == null) {
                return null;
            } else if (lowerKey == null) {
                return this.get(upperKey);
            } else if (upperKey == null)
                return this.get(lowerKey);

            // Get the various values for interpolation
            Double lowerValue = this.get(lowerKey);
            Double keyToLower = key - lowerKey;
            Double upperToLower = upperKey - lowerKey;
            Double inverse = keyToLower <= 0 || upperToLower <= 0 ? 0 : keyToLower / upperToLower;
            return (this.get(upperKey) - lowerValue) * inverse + lowerValue;
        } else
            return value;
    }
}
