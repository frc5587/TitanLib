package org.frc5587.lib.control;

import org.frc5587.lib.MathHelper;

import edu.wpi.first.wpilibj.Joystick;

public class DeadbandJoystick extends Joystick {
    private static double DEFAULT_DEADBAND = 0.05; 
    private final double deadbandCutoff;
    private final double exponent;

    /**
     * Construct an instance of a joystick which ignores axis inputs that are below
     * the specified deadband cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandJoystick(int port, double exp) {
        this(port, exp, DEFAULT_DEADBAND);
    }

    /**
     * Construct an instance of a joystick which ignores axis inputs that are below
     * a default cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandJoystick(int port) {
        this(port, 1);
    }

    public DeadbandJoystick(int port, double exp, double deadband) {
        super(port);
        this.exponent = exp;
        this.deadbandCutoff = deadband;
    }

    @Override
    public double getRawAxis(int axis) {
        // Apply deadband when getting axis so all axes will have deadbanded signal
        double value = super.getRawAxis(axis);
        value = Math.copySign(Math.pow(Math.abs(value), exponent), value);

        return MathHelper.deadband(value, deadbandCutoff, 1);
    }
}