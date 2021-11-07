package org.frc5587.lib.advanced;

import java.util.function.BiFunction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Notifier;

public class AddressableLEDController {
    private int port;
    private int length;
    private int step = 0;

    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;

    private Notifier ledNotifier;

    /***
     * Initialized the controller. This controller is meant to control individually
     * addressable LEDs. It works for WS2812 architecture, but it should work for
     * WS2812B as well.
     * 
     * @param port   PWM header port number
     * @param length number of LEDs on the strip
     */
    public AddressableLEDController(int port, int length) {
        this.port = port;
        this.length = length;

        leds = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        leds.setLength(length);
        leds.setData(ledBuffer);
        leds.start();
    }

    /**
     * This stops the LED step handler if it is running.
     */
    public void stopLEDStepHandlerNotifier() {
        if (ledNotifier != null) {
            ledNotifier.stop();
            ledNotifier.close();
            ledNotifier = null;
        }
    }

    /**
     * Starts the notifier handler that will run the custom function. It is
     * recommended to have the period be 0.02, meaning it will run 50 times a
     * second, but any number should work just fine.
     * 
     * @param ledStepperFunction custom LED controller function <step number,
     *                           LEDbuffer, LEDbuffer>
     * @param period             time between each run of the function (recommended
     *                           as 0.02)
     */
    public void startLEDStepHandlerNotifier(
            BiFunction<Integer, AddressableLEDBuffer, AddressableLEDBuffer> ledStepperFunction, double period) {
        stopLEDStepHandlerNotifier();

        ledNotifier = new Notifier(() -> {
            LEDStepHandler(ledStepperFunction);
        });
        ledNotifier.startPeriodic(period);
    }

    /**
     * This is a uniform way to handle most simple custom functions (like a moving
     * rainbow) to control the LEDs. Every time this is called, it raises the step
     * counter by 1.
     * 
     * @param ledStepperFunction custom LED controller function <step number,
     *                           LEDbuffer, LEDbuffer>
     */
    public void LEDStepHandler(BiFunction<Integer, AddressableLEDBuffer, AddressableLEDBuffer> ledStepperFunction) {
        ledBuffer = ledStepperFunction.apply(step, ledBuffer);
        updateLEDs();

        step++;
    }

    /**
     * Same as rainbow() except the rainbow is stretch out over the whole length of
     * the LED strip.
     * 
     * @param stepPeriod number of steps for the pattern to move a whole wavelength
     * @param step       the step number, each step will generally be 1/50th of a
     *                   second [given by handler]
     * @param buffer     the LED buffer, it is modified then returned [given by
     *                   handler]
     * @return the modified LED buffer
     */
    public AddressableLEDBuffer stretchRainbow(int stepPeriod, int step, AddressableLEDBuffer buffer) {
        return rainbow(stepPeriod, length, length, step, buffer);
    }

    /**
     * And example of a custom LED step function. It is a moving rainbow has a
     * custom wavelength and movement "speed."
     * 
     * @param stepPeriod number of steps for the pattern to move a whole wavelength
     * @param wavelength length (in LEDs) of the wave, so number of LEDS between the
     *                   two red points (or any other color)
     * @param length     length of LED strip
     * @param step       the step number, each step will generally be 1/50th of a
     *                   second [given by handler]
     * @param buffer     the LED buffer, it is modified then returned [given by
     *                   handler]
     * @return the modified LED buffer
     */
    public static AddressableLEDBuffer rainbow(int stepPeriod, int wavelength, int length, int step,
            AddressableLEDBuffer buffer) {
        int degrees = 360;
        int saturation = 1;
        int value = 1;

        for (int i = 0; i < length; i++) {
            int shiftedI = (i + (step * wavelength / stepPeriod)) % length;
            int hue = (shiftedI * degrees / wavelength) % degrees;

            buffer.setHSV(i, hue, saturation, value);
        }
        return buffer;
    }

    /**
     * Sets the entire strip to one color using the RGB notation.
     * 
     * @param r red value [0, 255]
     * @param g blue value [0, 255]
     * @param b blue value [0, 255]
     */
    public void setColorRGB(int r, int g, int b) {
        for (int i = 0; i < length; i++) {
            ledBuffer.setRGB(i, r, g, b);
        }

        updateLEDs();
    }

    /**
     * Sets a single LED on the strip to an RGB color.
     * 
     * @param index location of LED, 0 is closest to the RIO
     * @param r     red value [0, 255]
     * @param g     blue value [0, 255]
     * @param b     blue value [0, 255]
     */
    public void setLEDColorRBG(int index, int r, int g, int b) {
        ledBuffer.setRGB(index, r, g, b);

        updateLEDs();
    }

    /**
     * Updates all the LEDs to whatever is in the buffer (ledBuffer)
     */
    public void updateLEDs() {
        leds.setData(ledBuffer);
    }

    /**
     * Gets the PWM port number being used.
     * 
     * @return port number
     */
    public int getPort() {
        return port;
    }

    /**
     * Gets the length of the LED strip.
     * 
     * @return number of LEDs
     */
    public int getLength() {
        return length;
    }
}
