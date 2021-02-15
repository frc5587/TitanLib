package org.frc5587.lib.control;

import org.frc5587.lib.MathHelper;

import edu.wpi.first.wpilibj.XboxController;

public class DeadbandXboxController extends XboxController {
    private final double deadbandCutoff;

    /**
     * Construct an instance of an Xbox controller which ignores axis inputs that
     * are below the specified deadband cutoff.
     *
     * @param port           The port on the Driver Station that the joystick is
     *                       plugged into.
     * @param deadbandCutoff amount of deadband to apply to each axis
     */
    public DeadbandXboxController(int port, double deadbandCutoff) {
        super(port);
        this.deadbandCutoff = deadbandCutoff;
    }

    /**
     * Construct an instance of an Xbox controller which ignores axis inputs that
     * are below a default cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandXboxController(int port) {
        this(port, 0.1);
    }

    @Override
    public double getRawAxis(int axis) {
        // Apply deadband when getting axis so all axes will have deadbanded signal
        return MathHelper.deadband(super.getRawAxis(axis), deadbandCutoff, 1);
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