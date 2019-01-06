package org.frc5587.lib;

public class MathHelper {
    /**
     * Limits values to the given range, returning min or max if the value is lesser
     * or greater than either respectively, or the value parameter itself if it is
     * within the range. For a remapping between a range rather than applying simple
     * limits, see the {@link #deadband(double, double, double)} method.
     * 
     * @param value the value to fit in the given range of min and max
     * @param min   the minimum accepted value for the range - will be returned if
     *              the value is less than it
     * @param max   the maximum accepted value for the range - will be returned if
     *              the value is greater than it
     * @return double after fitting the value within the range
     */
    public static double limit(double value, double min, double max) {
        if (value > max) {
            return max;
        }
        if (value < min) {
            return min;
        }
        return value;
    }

    /**
     * Remaps a value to a range between the deadbandCutoff and max parameters. This
     * method does not validate that the value parameter is less than the max
     * parameter, so ensure that this is true on your own if this is functionality
     * that you desire.
     * 
     * @param value          the value to remap
     * @param deadbandCutoff the value below which all input will be counted as zero
     *                       (start of remapped range)
     * @param max            the maximum value that can be found in the value
     *                       parameter before remapping (should always be positive)
     * @return value after being remapped to range set by deadbandCutoff and max
     */
    public static double deadband(double value, double deadbandCutoff, double max) {
        if (Math.abs(value) < deadbandCutoff) {
            return 0.0;
        } else {
            return (value - (Math.abs(value) / value * deadbandCutoff)) / (max - deadbandCutoff);
        }
    }
}