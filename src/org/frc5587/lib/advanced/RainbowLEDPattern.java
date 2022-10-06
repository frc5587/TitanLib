package org.frc5587.lib.advanced;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainbowLEDPattern implements CustomLEDPattern {
    private static final int maxHue = 180;
    private static final int maxSaturation = 255;
    private static final double stepTime = 0.02;
    private double idx = 0;
    private int speed, wavelength, brightness;
    private int[] allLeds;

    /**
     * Creates the rainbow pattern LED controller. You can set `waveLength` and `length` to the same value if you want to "stretch" the rainbow across the entire strip (it looks kinda fire)
     * 
     * @param speed how fast the pattern travels across the strip, in pixels per second
     * @param wavelength length of the wave, so distance (in pixels) from red to red
     * @param length length of LED strip, in pixels
     * @param brightness from 0 to 255, its the brightness of the LED strip
     */
    public RainbowLEDPattern(int speed, int wavelength, int length, int brightness) {
        this.speed = speed;
        this.wavelength = wavelength;
        this.brightness = brightness;

        allLeds = new int[length];
        for (int i = 0; i < allLeds.length; i++) {
            allLeds[i] = (i * maxHue / this.wavelength) % maxHue;
        }
    }

    @Override
    public AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer) {
        for (int i = 0; i < allLeds.length; i++) {
            ledBuffer.setHSV(i, allLeds[(i + (int) idx) % allLeds.length], maxSaturation, brightness);
        }
        
        idx += stepTime * speed;

        return ledBuffer;
    }
}
