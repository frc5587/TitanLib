package org.frc5587.lib.control;

import org.frc5587.lib.MathHelper;

import edu.wpi.first.wpilibj.Joystick;

public class DeadbandJoystick extends Joystick {
    private final double deadbandCutoff;

    /**
     * Construct an instance of a joystick which ignores axis inputs that are below
     * the specified deadband cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandJoystick(int port, double deadband) {
        super(port);
        this.deadbandCutoff = deadband;
    }

    /**
     * Construct an instance of a joystick which ignores axis inputs that are below
     * a default cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandJoystick(int port) {
        this(port, 0.1);
    }

    @Override
    public double getRawAxis(int axis) {
        // Apply deadband when getting axis so all axes will have deadbanded signal
        return MathHelper.deadband(super.getRawAxis(axis), deadbandCutoff, 1);
    }
}