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

    public double curveY() {
        return 1 / this.deadbandY();
    }

    public double curveX() {
        return 1 / this.deadbandX();
    }
}