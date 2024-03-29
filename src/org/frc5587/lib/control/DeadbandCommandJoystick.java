package org.frc5587.lib.control;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DeadbandCommandJoystick extends CommandJoystick {
    private static double DEFAULT_DEADBAND = 0.05; 
    private final double deadbandCutoff;
    private final double exponent;

    public static class Curve {
        public static final double LINE_SLOPE = 0.3623;
        public static final double X_5_SLOPE = 0.638;
        public static final double DAMPEN_RATIO = 0.25;

        /**
         * Curves the value so it is shallow for a bit, but quickly increases
         * 
         * https://www.desmos.com/calculator/ehg1nvfznl
         * 
         * @param value expected between [-1, 1]
         * @return the curved value
         */
        public static double curve(double value) {
            return (LINE_SLOPE * value) + (X_5_SLOPE * Math.pow(value, 5));
        }

        public static double dampen(double value, double dampenAmount) {
            return value - (value * Math.abs(dampenAmount) * DAMPEN_RATIO);
        }
    }

    /**
     * Construct an instance of a joystick which ignores axis inputs that are below
     * the specified deadband cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandCommandJoystick(int port, double exp) {
        this(port, exp, DEFAULT_DEADBAND);
    }

    /**
     * Construct an instance of a joystick which ignores axis inputs that are below
     * a default cutoff.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public DeadbandCommandJoystick(int port) {
        this(port, 1);
    }

    public DeadbandCommandJoystick(int port, double exp, double deadband) {
        super(port);
        this.exponent = exp;
        this.deadbandCutoff = deadband;
    }

    @Override
    public double getRawAxis(int axis) {
        // Apply deadband when getting axis so all axes will have deadbanded signal
        double value = super.getRawAxis(axis);
        // value = Math.copySign(Math.pow(Math.abs(value), exponent), value);

        return MathUtil.applyDeadband(value, deadbandCutoff);
    }

    public double getXExp() {
        return Math.pow(getX(), exponent);
    }

    public double getYExp() {
        return Math.pow(getY(), exponent);
    }

    public double getXCurved() {
        return Curve.curve(getX());
    }

    public double getYCurved() {
        return Curve.curve(getY());
    }

    public double getXCurveDampened() {
        return Curve.curve(Curve.dampen(getX(), getY()));
    }

    public Translation2d getPosition() {
        return new Translation2d(getX(), getY());
    }
}