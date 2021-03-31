  
package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5587.lib.pid.JRAD;

import java.util.function.DoubleSupplier;


public class ShooterFeedbackController {
  private JRAD JRADConstants;
  private double lastOutput = 0;
  private double setpointVelocityRPS;
  private static double loopTime = 0.02;

  // private double errorThreshold = 0.98;

  private DoubleSupplier motorVelocitySupplier;

  /**
   * This builds a controller based on
   * https://www.team254.com/frc-day-12-13-build-blog/ Its controls the shooter to
   * anticipate the drop in speed from shooting the ball.
   * 
   * @param JRADConstants the JRAD constants 
   * @param motorVelocitySupplier supplier to get motor velocity in RPS
   */
  public ShooterFeedbackController(JRAD JRADConstants, DoubleSupplier motorVelocitySupplierRPS) {
    this.JRADConstants = JRADConstants;
    this.motorVelocitySupplier = motorVelocitySupplierRPS;
  }

  /**
   * @param currentVelocityRPS the current velocity - ROTATIONS PER SECOND
   * @return the voltage to set the motor to
   */
  public double calculate(double currentVelocityRPS) {
    this.lastOutput = (JRADConstants.kF * setpointVelocityRPS) + lastOutput
        + (JRADConstants.kJ * loopTime * ((JRADConstants.kLoadRatio * setpointVelocityRPS) - currentVelocityRPS));
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

  /*
   * Sends debug info to Smart Dashboard
   */
  public void sendDebugInfo() {
    SmartDashboard.putNumber("Shooter Last Output (V)", lastOutput);
    SmartDashboard.putNumber("Shooter Current Velocity (RPS)", motorVelocitySupplier.getAsDouble());
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
    return (motorVelocitySupplier.getAsDouble() > setpointVelocityRPS * JRADConstants.kLoadRatio);
  }

  public void reset() {
    lastOutput = 0;
  }
}