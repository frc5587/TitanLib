package org.frc5587.lib.control;

/**
 * Cycle
 */
public interface Cycle<T> {
    public T reset();
    public T prev();
    public T next();
    public T current();
    public T getValueForStep(int step);
}