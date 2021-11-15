package org.frc5587.lib.advanced;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class RainbowLEDPattern implements CustomLEDPattern {
    private static final int maxHue = 180;
    private static final int maxSaturation = 255;
    private static final int maxValue = 255;
    private int lastHue = 0;
    private int stepPeriod, wavelength, length, hsvValue;

    public RainbowLEDPattern(int stepPeriod, int wavelength, int length, int hsvValue) {
        this.stepPeriod = stepPeriod;
        this.wavelength = wavelength;
        this.length = length;
        this.hsvValue = hsvValue;
    }

    @Override
    public AddressableLEDBuffer step(int stepNumber, AddressableLEDBuffer ledBuffer) {
        lastHue += (maxHue / stepPeriod) % maxHue;
        for (int i = 0; i < length; i++) {
            int hue = (lastHue + (maxHue / wavelength)) % maxHue;
            ledBuffer.setHSV(i, hue, maxSaturation, hsvValue);
        }

        return ledBuffer;
    }
}
