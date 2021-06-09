
package org.frc5587.lib.controllers;

import java.util.function.DoubleSupplier;

public class JRADShooterController {
    private double kF;
    private double kJ;
    private double kLoadRatio;
    private double period;
    private double setpointVelocityRPS;
    private double lastOutput = 0;

    // private double errorThreshold = 0.98;

    private DoubleSupplier motorVelocitySupplier;

    /**
     * This builds a controller based on
     * https://www.team254.com/frc-day-12-13-build-blog/ Its controls the shooter to
     * anticipate the drop in speed from shooting the ball.
     * 
     * @param kF         arbitrary constant - 0.001 suggested defaults
     * @param kJ         arbitrary constant - 0
     * @param kLoadRatio arbitrary constant - 1
     */
    public JRADShooterController(double kF, double kJ, double kLoadRatio) {
        this(kF, kJ, kLoadRatio, 0.02);
    }

    /**
     * This builds a controller based on
     * https://www.team254.com/frc-day-12-13-build-blog/ Its controls the shooter to
     * anticipate the drop in speed from shooting the ball.
     * 
     * @param kF         arbitrary constant - 0.001 suggested defaults
     * @param kJ         arbitrary constant - 0
     * @param kLoadRatio arbitrary constant - 1
     * @param period     the loop time for each run of the controller in second
     */
    public JRADShooterController(double kF, double kJ, double kLoadRatio, double period) {
        this.kF = kF;
        this.kJ = kJ;
        this.kLoadRatio = kLoadRatio;
        this.period = period;
    }

    public void setVelocitySupplier(DoubleSupplier velocitySupplier) {
        motorVelocitySupplier = velocitySupplier;
    }

    /**
     * @param currentVelocityRPS the current velocity - ROTATIONS PER SECOND
     * @return the voltage to set the motor to
     */
    public double calculate(double currentVelocityRPS) {
        this.lastOutput = (kF * setpointVelocityRPS) + lastOutput
                + (kJ * period * ((kLoadRatio * setpointVelocityRPS) - currentVelocityRPS));
        return this.lastOutput;
    }

    public double calculate() {
        return calculate(motorVelocitySupplier.getAsDouble());
    }

    /**
     * Calculates the voltage to set the motor to
     * 
     * @param currentVelocityRPS  current speed of the shooter - ROTATIONS PER
     *                            SECOND
     * @param setpointVelocityRPS setpoint speed - ROTATIONS PER SECOND
     * @return voltage - VOLTS
     */
    public double calculate(double currentVelocityRPS, double setpointVelocityRPS) {
        setSetpoint(setpointVelocityRPS);
        return calculate(currentVelocityRPS);
    }

    /**
     * Sets the setpoint for the PID
     * 
     * @param setpointVelocityRPS setpoint - ROTATIONS PER SECOND
     */
    public void setSetpoint(double setpointVelocityRPS) {
        this.setpointVelocityRPS = setpointVelocityRPS;
    }

    /**
     * Sets the setpoints, the calculates the PID based on the current speed of the
     * shooter, which is retrieved via the DoubleSupplier
     * 
     * @param setpointVelocityRPS setpoint - ROTATIONS PER SECOND
     * @return voltage to set the shooter - VOLTS
     */
    public double setSetpointAndCalculate(double setpointVelocityRPS) {
        setSetpoint(setpointVelocityRPS);
        return calculate();
    }

    /**
     * Get the current setpoint of the shooter
     * 
     * @return setpoint - ROTATIONS PER SECOND
     */
    public double getSetpoint() {
        return setpointVelocityRPS;
    }

    /**
     * Because this overshoots the setpoint (as designed), we should only check if
     * it is above the setpoint. Once a ball shoots the speed will drop, but the
     * speed should always stay above the setpoint
     * 
     * @return true if the speed is above the setpoint
     */
    public boolean atSetpoint() {
        return (motorVelocitySupplier.getAsDouble() > setpointVelocityRPS * kLoadRatio);
    }

    public void reset() {
        lastOutput = 0;
    }

    public double getF() {
        return kF;
    }

    public double getJ() {
        return kJ;
    }

    public double getLoadRatio() {
        return kLoadRatio;
    }

    public void setF(double kF) {
        this.kF = kF;
    }

    public void setJ(double kJ) {
        this.kJ = kJ;
    }

    public void setLoadRatio(double kLoadRatio) {
        this.kLoadRatio = kLoadRatio;
    }
}