package org.frc5587.lib;

/**
 * Collection of useful math functions not already provided by WPILib in
 * {@link edu.wpi.first.wpiutil.math.MathUtil}
 */
public class MathHelper {
    /**
     * Eliminates values that are not greater than the deadbandCutoff within the
     * given range <code>-1 * max</code> and <code>max</code>.
     * 
     * <p>
     * Note that all inputed values will be eliminated if the maximum possible
     * signal indicated by `max` is smaller than the deadband cutoff. Additonally,
     * the method itself does limit outputted signals to be within the maximum
     * range. If this is desired, use
     * {@link edu.wpi.first.wpiutil.math.MathUtil#clamp(double, double, double)}.
     * 
     * @param value          the value to remap
     * @param deadbandCutoff the value below which all input will be counted as zero
     *                       (start of remapped range)
     * @param max            the maximum value that can be found in the value
     *                       parameter before remapping (should always be positive)
     * @return value filtered based on the given deadband cutoff
     */
    public static double deadband(double value, double deadbandCutoff, double max) {
        if (Math.abs(value) < deadbandCutoff) {
            return 0.0;
        } else {
            return (value - Math.copySign(deadbandCutoff, value)) / (max - deadbandCutoff);
        }
    }

    /**
     * Maps the input x within range inMin to inMax to a new range between outMin
     * and outMax. See the
     * <a href="https://www.arduino.cc/reference/en/language/functions/math/map/">
     * Arduino <code>map()</code> function</a> for more information
     * 
     * @param x      the value to remap
     * @param inMin  the minimum potential of value x
     * @param inMax  the maximum potential value of x
     * @param outMin the minimum potential value of the new range
     * @param outMax the maximum potential value of the new range
     * @return the value found after remapping x from the old to new range
     */
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    /**
     * Maps the input x within range inMin to inMax to a new range between outMin
     * and outMax. See the
     * <a href="https://www.arduino.cc/reference/en/language/functions/math/map/">
     * Arduino function map()</a> for more information
     * 
     * @param x      the value to remap
     * @param inMin  the minimum potential of value x
     * @param inMax  the maximum potential value of x
     * @param outMin the minimum potential value of the new range
     * @param outMax the maximum potential value of the new range
     * @return the value found after remapping x from the old to new range
     */
    public static float map(float x, float inMin, float inMax, float outMin, float outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    /**
     * Maps the input x within range inMin to inMax to a new range between outMin
     * and outMax. See the
     * <a href="https://www.arduino.cc/reference/en/language/functions/math/map/">
     * Arduino function map()</a> for more information
     * 
     * @param x      the value to remap
     * @param inMin  the minimum potential of value x
     * @param inMax  the maximum potential value of x
     * @param outMin the minimum potential value of the new range
     * @param outMax the maximum potential value of the new range
     * @return the value found after remapping x from the old to new range
     */
    public static long map(long x, long inMin, long inMax, long outMin, long outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    /**
     * Maps the input x within range inMin to inMax to a new range between outMin
     * and outMax. See the
     * <a href="https://www.arduino.cc/reference/en/language/functions/math/map/">
     * Arduino function map()</a> for more information
     * 
     * @param x      the value to remap
     * @param inMin  the minimum potential of value x
     * @param inMax  the maximum potential value of x
     * @param outMin the minimum potential value of the new range
     * @param outMax the maximum potential value of the new range
     * @return the value found after remapping x from the old to new range
     */
    public static int map(int x, int inMin, int inMax, int outMin, int outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }
}