package org.frc5587.lib;

import edu.wpi.first.wpilibj.XboxController;

public class DeadbandXboxController extends XboxController {
    private double deadbandCutoff;

    /**
    * Construct an instance of a joystick. The joystick index is the USB port on the drivers
    * station.
    *
    * @param port The port on the Driver Station that the joystick is plugged into.
    */
    public DeadbandXboxController(final int port) {
        super(port);
        this.deadbandCutoff = 0.1;
    }

    /**
    * Construct an instance of a joystick. The joystick index is the USB port on the drivers
    * station.
    *
    * @param port The port on the Driver Station that the joystick is plugged into.
    * @param deadbandCutoff amount of deadband to apply to each axis
    */
    public DeadbandXboxController(final int port, final double deadbandCutoff) {
        super(port);
        this.deadbandCutoff = deadbandCutoff;
    }

    /**
    * Get the X axis value of the controller.
    *
    * @param hand Side of controller whose value should be returned.
    * @return The X axis value of the controller.
    */
    @Override
    public double getX(Hand hand) {
        return deadband(super.getX(hand));
    }

    /**
     * Get the Y axis value of the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The Y axis value of the controller.
     */
    @Override
    public double getY(Hand hand) {
        return deadband(super.getY(hand));
    }

    /**
     * Returns zero if the joystick is in the dead zone, else returns the joystick value as mapped
     * to a scale that starts at the end of the deadzone as zero in order to prevent jump from
     * zero to above the deadzone when the deadzone is left
     * 
     * @param joystickValue Value of joystick
     * @return The value of the joystick with the deadband applied
     */
    private double deadband(double joystickValue) {
        if (Math.abs(joystickValue) < deadbandCutoff) {
            return 0.0;
        } else {
            return (joystickValue - (Math.abs(joystickValue) / joystickValue * deadbandCutoff)) / (1 - deadbandCutoff);
        }
    }

    public boolean getTrigger(Hand hand) {
        return super.getTriggerAxis(hand) > deadbandCutoff;
    }
}