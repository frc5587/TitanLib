package org.frc5587.lib.advanced;

import java.util.function.BiFunction;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AddressableLEDController extends SubsystemBase {
    private int port;
    private int length;
    private int step = 0;
    private int maxBrightness = 255;

    private AddressableLED leds;
    private AddressableLEDBuffer ledBuffer;

    private BiFunction<Integer, AddressableLEDBuffer, AddressableLEDBuffer> ledStepperFunction;

    /**
     * Initialized the controller. This controller is meant to control individually
     * addressable LEDs. It works for WS2812 architecture, but it should work for
     * WS2812B as well.
     * 
     * @param port   PWM header port number
     * @param length number of LEDs on the strip
     * @param ledStepperFunction function to step the LED strip, its given the step and buffer, and is expected to return the modified buffer
     */
    public AddressableLEDController(int port, int length, BiFunction<Integer, AddressableLEDBuffer, AddressableLEDBuffer> ledStepperFunction) {
        this.port = port;
        this.length = length;
        this.ledStepperFunction = ledStepperFunction;

        leds = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        leds.setLength(length);
        leds.setData(ledBuffer);
        leds.start();
    }

    /**
     * Initialized the controller. This controller is meant to control individually
     * addressable LEDs. It works for WS2812 architecture, but it should work for
     * WS2812B as well.
     * 
     * @param port   PWM header port number
     * @param length number of LEDs on the strip
     * @param ledPattern this just gives a little more control than the ledStepperFunction
     */
    public AddressableLEDController(int port, int length, CustomLEDPattern ledPattern) {
        this(port, length, (BiFunction<Integer, AddressableLEDBuffer, AddressableLEDBuffer>) ledPattern::step); //this casting is necessary other java gets made and is like "its too vague!?!?!" cause technically ledPattern::step could be null, and then it wouldn't know which constructor to call
    }

    /**
     * Steps the pattern function and updates the LEDs
     */
    @Override
    public void periodic() {
        ledBuffer = ledStepperFunction.apply(step, ledBuffer);
        updateLEDs();

        step++;
    }

    /**
     * Sets the pattern that is run in periodic, it will update the LEDs on the next iteration of periodic()
     * 
     * @param ledPattern custom LED pattern object, use if you really need to
     *                   preserve variables between runs, otherwise use the other
     *                   version of this method to make things simpler
     */
    public void setPattern(CustomLEDPattern ledPattern) {
        ledStepperFunction = ledPattern::step;
    }

    /**
     * Sets the pattern that is run in periodic, it will update the LEDs on the next iteration of periodic()
     * 
     * @param ledStepperFunction custom LED controller function <step number,
     *                           LEDbuffer, LEDbuffer>
     */
    public void setPattern(BiFunction<Integer, AddressableLEDBuffer, AddressableLEDBuffer> ledStepperFunction) {
        this.ledStepperFunction = ledStepperFunction;
    }

    /**
     * Same as rainbow() except the rainbow is stretched out over the whole length
     * of the LED strip.
     * 
     * @param stepPeriod number of steps for the pattern to move a whole wavelength
     * @param step       the step number, each step will generally be 1/50th of a
     *                   second [given by handler]
     * @param buffer     the LED buffer, it is modified then returned [given by
     *                   handler]
     * @return the modified LED buffer
     */
    public AddressableLEDBuffer stretchRainbow(int stepPeriod, int step, AddressableLEDBuffer buffer) {
        return rainbow(stepPeriod, length, length, maxBrightness, step, buffer);
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
    public static AddressableLEDBuffer rainbow(int stepPeriod, int wavelength, int length, int value, int step,
            AddressableLEDBuffer buffer) {
        int degrees = 180;
        int saturation = 255;

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
