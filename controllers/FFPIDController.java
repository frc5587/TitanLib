package org.frc5587.lib.controllers;

public class FFPIDController {
    public final double kS;
    public final double kCos;
    public final double kG;
    public final double kV;
    public final double kA;

    /**
    * sets all given values for:
    * @param kS static gain
    * @param kCos gravity
    * @param kV velocity gain
    * @param kA acceleration gain
    */
    public FFPIDController(double kS, double kCos, double kG, double kV, double kA) {
        this.kS = kS;
        this.kCos = kCos;
        this.kG = kG;
        this.kV = kV;
        this.kA = kA;
    }

    /**  
    * use this for an elevator
    * @param velocity the subsystem velocity in radians per second 
    * (account for encoder counts per revolution and gearing BEFORE passing this parameter) 
    * @param acceleration acceleration in radians per second squared 
    */
    public double calculateElevator(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kG + kV * velocity + kA * acceleration;
    }

    /**  
    * use this for an elevator, or any FPIDSubsystem that does not require an angle gain.
    * @param positionRadians the angle of the arm in radians
    * @param velocity the subsystem velocity in radians per second 
    * (account for encoder counts per revolution and gearing BEFORE passing this parameter) 
    * @param acceleration acceleration in radians per second squared 
    */
    public double calculateArm(double positionRadians, double velocity, double acceleration) {
        return kS * Math.signum(velocity)
            + kCos * Math.cos(positionRadians)
            + kV * velocity
            + kA * acceleration;
    }

    /**  
    * use this for an elevator, or any FPIDSubsystem that does not require an angle gain.
    * @param velocity the subsystem velocity in radians per second 
    * (account for encoder counts per revolution and gearing BEFORE passing this parameter) 
    * @param acceleration acceleration in radians per second squared 
    */
    public double calculateSimpleMotor(double velocity, double acceleration) {
        return kS * Math.signum(velocity) + kV * velocity + kA * acceleration;
    }
}
