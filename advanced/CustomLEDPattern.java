package org.frc5587.lib.advanced;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

/**
 * This is an alternative way to write a custom LED pattern, instead of a
 * function that can't preserve its variables, you can instead use a class with
 * a step function.
 */
public interface CustomLEDPattern {
    /**
     * Called each time to update the LEDs, returns the modified LED buffer
     * 
     * @param stepNumber step number it on, each step will generally represent 0.02
     *                   seconds
     * @return modified LED buffer to update the LEDs
     */
    public abstract AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer);
}