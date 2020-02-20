package org.frc5587.lib.control;

import edu.wpi.first.wpilibj.Joystick;

import org.frc5587.lib.MathHelper;

public class DeadbandJoystick extends Joystick {
    public DeadbandJoystick(int port) {
        super(port);
    }

    public double deadbandX() {
        return MathHelper.deadband(this.getX(), 0.1, 1);
    }

    public double deadbandY() {
        return MathHelper.deadband(this.getY(), 0.1, 1);
    }

    public double deadbandZ() {
        return MathHelper.deadband(this.getZ(), 0.1, 1);
    }

    public double curveY() {
        double y = this.deadbandY();
        return Math.copySign(y * y, y);
    }

    public double curveX() {
        double x = this.deadbandX();
        return Math.copySign(x * x, x);
    }

    public double curveZ() {
        double z = this.deadbandZ();
        return Math.copySign(z * z, z);
    }

    public double curveYnd() {
        double y = this.getY();
        return Math.copySign(y * y, y);
    }

    public double curveXnd() {
        double x = this.getX();
        return Math.copySign(x * x, x);
    }

    public double curveZnd() {
        double z = this.getZ();
        return Math.copySign(z * z, z);
    }
}