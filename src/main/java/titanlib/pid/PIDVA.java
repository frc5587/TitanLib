package org.frc5587.lib.pid;

/**
 * PIDVA
 */
public class PIDVA extends PID {
    public final double kV;
    public final double kA;

    public PIDVA(double kP, double kI, double kD, double kV, double kA) {
        super(kP, kI, kD);
        this.kV = kV;
        this.kA = kA;
    }
}