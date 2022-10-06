package org.frc5587.lib.control;

public class Cycler<T> implements Cycle<T> {
    private T[] enumValues;
    private boolean loop;
    private int currentIndex, startIndex;

    /**
     * Cycles through any array of ordered values. Unless shouldLoop is set to
     * true, when encountering a step that goes beyond an upper or lower bound, it
     * will not cross that bound Note: <strong>values must be ordered</strong>.
     * 
     * @param valuesToCycle values that are intended to be cycled through
     *                          without looping
     */
    public Cycler(T[] valuesToCycle) {
        this(valuesToCycle, false, 0);
    }

    /**
     * Cycles through any array of ordered values. Unless shouldLoop is set to
     * true, when encountering a step that goes beyond an upper or lower bound, it
     * will not cross that bound Note: <strong>values must be ordered</strong>.
     * 
     * @param valuesToCycle values that are intended to be cycled through
     *                          without looping
     * @param shouldLoop        whether or not the object should loop if a step will
     *                          go beyond a boundary
     */
    public Cycler(T[] valuesToCycle, boolean shouldLoop) {
        this(valuesToCycle, shouldLoop, 0);
    }

    /**
     * Cycles through any array of ordered values. Unless shouldLoop is set to
     * true, when encountering a step that goes beyond an upper or lower bound, it
     * will not cross that bound Note: <strong>values must be ordered</strong>.
     * 
     * @param valuesToCycle values that are intended to be cycled through
     *                          without looping
     * @param shouldLoop        whether or not the object should loop if a step will
     *                          go beyond a boundary
     * @param startIndex        index used to start and reset the cycle
     */
    public Cycler(T[] valuesToCycle, boolean shouldLoop, int startIndex) {
        this.enumValues = valuesToCycle;
        this.loop = shouldLoop;
        this.startIndex = startIndex;
        this.currentIndex = startIndex;
    }

    /**
     * Resets the current index to startIndex, which is 0 by default. After
     * resetting, it will return the object found at the new position.
     * 
     * @return the value found after resetting
     */
    public T reset() {
        currentIndex = startIndex;
        return current();
    }

    /**
     * Cycles one step backwards and returns the value at that point
     * <strong>according to the original order of the values</strong>. The
     * cycle will loop if the shouldLoop parameter was set to true when the class
     * was instantiated and a lower or upper bound is reached.
     * 
     * @return the value after cycling backwards by one step
     */
    public T prev() {
        return getValueForStep(-1);
    }

    /**
     * Function to return the value at the current step in the cycle.
     * 
     * @return the current value
     */
    public T current() {
        return enumValues[currentIndex];
    }

    /**
     * Cycles one step forwards and returns the value at that point
     * <strong>according to the original order of the values</strong>. The
     * cycle will loop if the shouldLoop parameter was set to true when the class
     * was instantiated and a lower or upper bound is reached.
     * 
     * @return the value after cycling forwards by one step
     */
    public T next() {
        return getValueForStep(1);
    }

    /**
     * Cycles any number of steps and then returns the value at the new
     * position. The cycle will loop if the shouldLoop parameter was set to true
     * when the class was instantiated and a lower or upper bound is reached.
     * 
     * @param step how far to step in the array of values
     * @returns the value found after stepping the amount step dictates
     */
    public T getValueForStep(int step) {
        if (step == 0) {
            return enumValues[currentIndex];
        } else {
            int newIndex = currentIndex + step;
            if (loop) {
                // If it loops, wrap around when faced with the upper or lower bound
                int loopedIndex = newIndex % enumValues.length;

                if (newIndex < 0) {
                    currentIndex = enumValues.length + loopedIndex;
                    return enumValues[enumValues.length + loopedIndex];
                } else {
                    currentIndex = loopedIndex;
                    return enumValues[loopedIndex];
                }
            } else {
                // If it does not loop, simply stop at the upper or lower bound
                if (newIndex >= enumValues.length || newIndex < 0) {
                    // Return the previousValue, as bound has been encountered
                    return enumValues[currentIndex];
                } else {
                    // Otherwise return the value yielded by stepping
                    currentIndex = newIndex;
                    return enumValues[newIndex];
                }
            }
        }
    }
}