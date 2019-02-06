package org.frc5587.lib.pid;

/**
 * FPID
 */
public class FPID extends PID{
    public final double kF;

    public FPID(double kP, double kI, double kD, double kF) {
        super(kP, kI, kD);
        this.kF = kF;
    }
}