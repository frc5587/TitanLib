
package org.frc5587.lib.controllers;

import java.util.function.DoubleSupplier;

/**
 * This is a shooter controller designed for used with a fixed angle shooter,
 * hence its use in {@link FixedHoodedShooterBase}
 * 
 * It uses the UNP regression model (show here:
 * https://www.desmos.com/calculator/yqs3muc58d) to determine the proper
 * velocity of the shooter flywheels based on the distance it is from the
 * shooter target. It also uses the JRAD controller (based on:
 * https://www.team254.com/frc-day-12-13-build-blog/) to control the voltage of
 * the motors based on the target velocity and current velocity. When properly
 * tuned, this will quickly account and anticipate the drop in velocity from
 * shooting a ball
 */
public class UnifiedShooterController {
    private static final double DEFAULT_LOOP_TIME = 0.02;

    private double kF;
    private double kJ;
    private double kU;
    private double kN;
    private double kP;
    private double period;
    private double setpointVelocityRPS;
    private double targetDistance;
    private double lastOutput = 0;

    // private double errorThreshold = 0.98;

    private DoubleSupplier motorVelocitySupplier;

    /**
     * This builds the controller using the default period time
     * 
     * @param kF arbitrary constant - 0.001 suggested defaults
     * @param kJ arbitrary constant - 0
     * @param kU regression constant
     * @param kN regression constant
     * @param kP regression constant
     */
    public UnifiedShooterController(double kF, double kJ, double kU, double kN, double kP) {
        this(kF, kJ, kU, kN, kP, DEFAULT_LOOP_TIME);
    }

    /**
     * The main feature of this is the UNP regression model, shown in this Desmos
     * graph, https://www.desmos.com/calculator/yr1pxmhu0g. The purple line in the
     * regression curve and the orange dots are measured data. Essentially, you give
     * this the distance from the target, and it does the UNP regression and then
     * passes that output into the JRAD controller.
     * 
     * The JRAD controller, similar to a PID controller, based on
     * https://www.team254.com/frc-day-12-13-build-blog/, controls the shooter to
     * anticipate the drop in speed from shooting the ball. Except this doesn't use
     * a loadRatio because all of that is handled in the UNP regression model.
     * 
     * @param kF     arbitrary constant - 0.001 suggested defaults
     * @param kJ     arbitrary constant - 0
     * @param kU     regression constant
     * @param kN     regression constant
     * @param kP     regression constant
     * @param period the loop time of controller, should almost always be 0.02s
     *               (1/50)
     */
    public UnifiedShooterController(double kF, double kJ, double kU, double kN, double kP, double period) {
        this.kF = kF;
        this.kJ = kJ;
        this.kU = kU;
        this.kN = kN;
        this.kP = kP;
        this.period = period;
    }

    /**
     * Sets the velocity supplier for the shooter flywheel.
     * 
     * @param velocitySupplierRPS supplies flywheel velocity - ROTATIONS PER SECOND
     */
    public void setVelocitySupplier(DoubleSupplier velocitySupplierRPS) {
        motorVelocitySupplier = velocitySupplierRPS;
    }

    /**
     * Given distance it returns the proper angular velocity (RPS) to spin up to
     * 
     * @param distance distance from target - METERS
     * @return angular velocity - ROTATIONS PER SECOND
     */
    public double calculateUNP(double distance) {
        return (kU * distance) + (kN / (distance - kP));
    }

    /**
     * Given angular velocity outputs voltage to set shooter motors to.
     * 
     * @param velocity angular velocity of flywheel - ROTATIONS PER SECOND
     * @return voltage to set motor(s) to - VOLTS
     */
    public double calculateVoltage(double velocity) {
        return (kF * setpointVelocityRPS) + lastOutput + (kJ * period * ((setpointVelocityRPS) - velocity));
    }

    /**
     * Calculates the voltage to set the motors to given current velocity of the
     * motors.
     * 
     * @param motorVelocityRPS current velocity of the shooter motor(s) - ROTATIONS
     *                         PER SECOND
     * @return voltage to set motor(s) to - VOLTS
     */
    public double calculate(double motorVelocityRPS) {
        this.setpointVelocityRPS = calculateUNP(this.targetDistance);
        this.lastOutput = calculateVoltage(motorVelocityRPS);
        return this.lastOutput;
    }

    /**
     * Calculates proper voltage with all known data and pulls the current velocity
     * of the motor.
     * 
     * @return voltage to set motor(s) to - VOLTS
     */
    public double calculate() {
        return calculate(motorVelocitySupplier.getAsDouble());
    }

    /**
     * Calculates the voltage to set the motor to
     * 
     * @param motorVelocityRPS   current velocity of the shooter motor(s) -
     *                           ROTATIONS PER SECOND
     * @param distanceFromTarget distance from target - METERS
     * @return voltage to set motor(s) to - VOLTS
     */
    public double setDistanceAndCalculate(double motorVelocityRPS, double distanceFromTarget) {
        setDistance(distanceFromTarget);
        return calculate(motorVelocityRPS);
    }

    /**
     * Sets the distance from the shooter target and then calculates the proper
     * voltage to set the motors to.
     * 
     * @param distanceFromTarget distance from the shooter target - METERS
     * @return voltage to set the shooter - VOLTS
     */
    public double setDistanceAndCalculate(double distanceFromTarget) {
        setDistance(distanceFromTarget);
        return calculate();
    }

    /**
     * Checks if the shooter is spinning fast enough
     * 
     * @return true if the speed is above the setpoint
     */
    public boolean atSetpoint() {
        return (motorVelocitySupplier.getAsDouble() > setpointVelocityRPS);
    }

    /**
     * Get the current setpoint of the shooter
     * 
     * @return setpoint - ROTATIONS PER SECOND
     */
    public double getSetpoint() {
        return setpointVelocityRPS;
    }

    public double getLastOutput() {
        return lastOutput;
    }

    /**
     * Sets the distance from the shooter target.
     * 
     * @param distance distance from target - METERS
     */
    public void setDistance(double distanceFromTarget) {
        this.targetDistance = distanceFromTarget;
    }

    public double getDistance() {
        return targetDistance;
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

    public double getU() {
        return kU;
    }

    public double getN() {
        return kN;
    }

    public double getP() {
        return kP;
    }

    public void setF(double kF) {
        this.kF = kF;
    }

    public void setJ(double kJ) {
        this.kJ = kJ;
    }

    public void setU(double kU) {
        this.kU = kU;
    }

    public void setN(double kN) {
        this.kN = kN;
    }

    public void setP(double kP) {
        this.kP = kP;
    }
}