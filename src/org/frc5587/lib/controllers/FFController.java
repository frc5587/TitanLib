package org.frc5587.lib.controllers;

@Deprecated
public class FFController {
    public final double kS;
    public final double kCos;
    public final double kG;
    public final double kV;
    public final double kA;

    /**
     * sets all given values for:
     * 
     * @param kS   static gain
     * @param kCos gravity
     * @param kG   gravity
     * @param kV   velocity gain
     * @param kA   acceleration gain
     */
    public FFController(double kS, double kCos, double kG, double kV, double kA) {
        this.kS = kS;
        this.kCos = kCos;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    /**
     * Use this for an elevator
     * all values should be in units that will be determined by how you caluclated
     * them in characterization.
     * 
     * @param velocity     the subsystem's velocity in units per second
     *                     (account for encoder counts per revolution and gearing
     *                     BEFORE passing this parameter)
     * @param acceleration acceleration in units per second squared
     */
    public double calculateElevator(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kG + kV * velocity + kA * acceleration;
    }

    public double calculateElevator(double velocity) {
        return calculateElevator(velocity, 0);
    }

    /**
     * Use this for an arm, or any FPIDSubsystem that does not require an angle gain
     * most values should be in units that will be determined by how you caluclated
     * them in characterization.
     * (unless specified otherwise)
     * 
     * @param positionRadians the angle of the arm in RADIANS
     * @param velocity        the subsystem velocity in units per second
     *                        (account for encoder counts per revolution and gearing
     *                        BEFORE passing this parameter)
     * @param acceleration    acceleration in units per second squared
     */
    public double calculateArm(double positionRadians, double velocity, double acceleration) {
        return kS * Math.signum(velocity)
                + kCos * Math.cos(positionRadians)
                + kV * velocity
                + kA * acceleration;
    }

    public double calculateArm(double positionRadians, double velocity) {
        return calculateArm(positionRadians, velocity, 0);
    }

    public double calculateArm(double positionRadians) {
        return calculateArm(positionRadians, 0, 0);
    }

    /**
     * Use this for a simple motor
     * all values should be in units that will be determined by how you caluclated
     * them in characterization.
     * 
     * @param velocity     the subsystem velocity in units per second
     *                     (account for encoder counts per revolution and gearing
     *                     BEFORE passing this parameter)
     * @param acceleration acceleration in units per second squared
     */
    public double calculateSimpleMotor(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }

    public double calculateSimpleMotor(double velocity) {
        return calculateSimpleMotor(velocity, 0);
    }
}
