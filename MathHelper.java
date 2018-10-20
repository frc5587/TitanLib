package org.frc5587.lib;


public class MathHelper{
    /**
     * Limit values to the given range.
     */
    public static double limit(double value, double min, double max){
        if (value > max) {
            return max;
        }
        if (value < min) {
            return min;
        }
        return value;
    }
}