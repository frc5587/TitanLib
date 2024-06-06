package org.frc5587.lib.control;

import org.frc5587.lib.math.MathHelper;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DeadbandCommandXboxController extends CommandXboxController {
    private static double DEFAULT_DEADBAND = 0.1;
    private final double deadbandCutoff;

    /**
     * Construct an instance of a CommandXboxController which ignores axis inputs that
     * are below the specified deadband cutoff.
     *
     * 
     * @param port           The port on the Driver Station that the joystick is
     *                       plugged into.
     * @param deadbandCutoff amount of deadband to apply to each axis
     */
    public DeadbandCommandXboxController(int port, double deadbandCutoff) {
        super(port);
        this.deadbandCutoff = deadbandCutoff;
    }

    /**
     * Construct an instance of an Xbox controller which ignores axis inputs that
     * are below a default cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandCommandXboxController(int port) {
        this(port, DEFAULT_DEADBAND);
    }

    @Override
    public double getRawAxis(int axis) {
        // Apply deadband when getting axis so all axes will have deadbanded signal
        return MathHelper.deadband(super.getRawAxis(axis), deadbandCutoff, 1);
    }

    /**
     * Whether the left trigger is currently depressed.
     * 
     * @return the state of the trigger.
     */
    public boolean getLeftTrigger() {
        return super.getLeftTriggerAxis() > deadbandCutoff;
    }

      /**
     * Whether the right trigger is currently depressed.
     * 
     * @return the state of the trigger.
     */
    public boolean getRightTrigger() {
        return super.getRightTriggerAxis() > deadbandCutoff;
    }

    @Override
    public double getLeftX() {
        return getRawAxis(XboxController.Axis.kLeftX.value);
    }

    @Override
    public double getLeftY() {
        return getRawAxis(XboxController.Axis.kLeftY.value);
    }

    @Override
    public double getRightX() {
        return getRawAxis(XboxController.Axis.kRightX.value);
    }

    @Override
    public double getRightY() {
        return getRawAxis(XboxController.Axis.kRightY.value);
    }
}
