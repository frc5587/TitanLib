package org.frc5587.lib.pid;

/**
 * 254's JRAD controller https://www.team254.com/frc-day-12-13-build-blog/
 */
public class JRAD {
    public final double kJ;          // 0.001        suggested defaults
    public final double kF;          // 0
    public final double kLoadRatio;  // 1

    public JRAD(double kJ, double kF, double kLoadRatio) {
        this.kJ = kJ;
        this.kF = kF;
        this.kLoadRatio = kLoadRatio;
    }
}