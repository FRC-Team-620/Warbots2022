package frc.robot.Util;

import java.util.TreeMap;

public class InterpolatingDoubleMap extends TreeMap<Double, Double> {
    /** 
     * Determines whether the specified key is between the 
     * lowest and highest keys in a non-empty tree map
     * 
     * @param key The key to check against
     * @return    Whether the key is inside the bounds
     */
    public boolean isKeyInBounds(Double key) {
        return !this.isEmpty() && key >= this.firstKey() && key <= this.lastKey();
    }

    /** 
     * Calculates the interpolated value of the specified 
     * key and returns the closest value if out of 
     * bounds or null if the tree map is empty
     * 
     * @param key The key to interpolate from
     * @return    The interpolated value from the key
     */
    public Double getInterpolated(Double key) {
        return this.getInterpolated(key, null);
    }

    /** 
     * Calculates the interpolated value of the specified
     * key and returns the default value if out of bounds
     * 
     * @param key          The key to interpolate from
     * @param defaultValue The value to return if out of bounds
     * @return             The interpolated value from the key
     */
    public Double getInterpolated(Double key, Double defaultValue) {
        // Return the default value if the specified key is out of bounds
        if (defaultValue != null && !this.isKeyInBounds(key))
            return defaultValue;
        // Return the associated value if a matching key happens to exist
        else if (this.containsKey(key))
            return this.get(key);
        else {
            // Get upper and lower keys for interpolation
            Double lowerKey = this.floorKey(key);
            Double upperKey = this.ceilingKey(key);

            // Return the nearest data point if at tree edge
            if (lowerKey == null && upperKey == null) {
                // Return null if tree is empty
                return null;
            } else if (lowerKey == null) {
                // Return lowest value if key is below the minimum limit
                return this.get(upperKey);
            } else if (upperKey == null)
                // Return highest value if key is above the maximum limit
                return this.get(lowerKey);

            // Get the various values for interpolation
            Double lowerValue = this.get(lowerKey);
            Double keyToLower = key - lowerKey;
            Double upperToLower = upperKey - lowerKey;

            // Verify that undefined and unwanted values are not produced in division
            Double inverse = keyToLower <= 0 || upperToLower <= 0 ? 0 : keyToLower / upperToLower;
            return (this.get(upperKey) - lowerValue) * inverse + lowerValue;
        }
    }
}
