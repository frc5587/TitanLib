package org.frc5587.lib.control;

import org.frc5587.lib.MathHelper;

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
        return MathHelper.deadband(super.getX(hand), deadbandCutoff, 1);
    }

    /**
     * Get the Y axis value of the controller.
     *
     * @param hand Side of controller whose value should be returned.
     * @return The Y axis value of the controller.
     */
    @Override
    public double getY(Hand hand) {
        return MathHelper.deadband(super.getY(hand), deadbandCutoff, 1);
    }

    /**
     * Whether the trigger is currently depressed.
     * 
     * @param hand Side of controller whose trigger should be checked.
     * @return the state of the trigger.
     */
    public boolean getTrigger(Hand hand) {
        return super.getTriggerAxis(hand) > deadbandCutoff;
    }
}