package org.frc5587.lib;

/**
 * This Code contains modified code that is part of WPILIB, accessible at 
 * https://github.com/wpilibsuite/allwpilib
 * 
 * Copyright (c) 2008-2018 FIRST. All Rights Reserved.
 * Open Source Software - may be modified and shared by FRC teams. The code must be 
 * accompanied by the FIRST BSD license file in the root directory of the project.
 */
public class TitanDrive{
    public static final double kDefaultQuickStopThreshold = 0.2;
    public static final double kDefaultQuickStopAlpha = 0.1;
    private static final double kDeadband = 0.01;
    private static final double kQuickTurnSafety = 0.1;

    private double m_quickStopThreshold = kDefaultQuickStopThreshold;
    private double m_quickStopAlpha = kDefaultQuickStopAlpha;
    private double m_quickStopAccumulator = 0.0;

    public TitanDrive(){}

    /**
     * Calling this sets squaredInputs to FALSE
     * @param throttle Sent to both sides, modified by the curve
     * @param turn Turning amount, added to one side and subtracted from the other
     */
    public DriveSignal arcadeDrive(double throttle, double turn){
        return arcadeDrive(throttle, turn, false);
    }

    /**
     * @param throttle Sent to both sides, modified by the curve
     * @param turn Turning amount, added to one side and subtracted from the other
     * @param squaredInputs Enabling makes the motor curve smoother closer to zero, bur removes linearity
     */
    public DriveSignal arcadeDrive(double throttle, double turn, boolean squaredInputs){
        throttle = applyDeadband(MathHelper.limit(throttle, -1.0, 1.0), kDeadband);
        turn = applyDeadband(MathHelper.limit(turn, -1.0, 1.0), kDeadband);

        if (squaredInputs) {
            throttle = Math.copySign(throttle * throttle, throttle);
            turn = Math.copySign(turn * turn, turn);
          }
      
          double left;
          double right;
      
          double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);
      
          if (throttle >= 0.0) {
            // First quadrant, else second quadrant
            if (turn >= 0.0) {
              left = maxInput;
              right = throttle - turn;
            } else {
              left = throttle + turn;
              right = maxInput;
            }
          } else {
            // Third quadrant, else fourth quadrant
            if (turn >= 0.0) {
              left = throttle + turn;
              right = maxInput;
            } else {
              left = maxInput;
              right = throttle - turn;
            }
          }
        return new DriveSignal(left, right);
    }

    public DriveSignal curvatureDrive(double throttle, double curve, boolean isQuickTurn) {
        throttle = applyDeadband(MathHelper.limit(throttle, -1.0, 1.0), kDeadband);
        curve = applyDeadband(MathHelper.limit(curve, -1.0, 1.0), kDeadband);

        double angularPower;
        boolean overPower;
    
        if (isQuickTurn && throttle <= kQuickTurnSafety) {
          if (Math.abs(throttle) < m_quickStopThreshold) {
            m_quickStopAccumulator = (1 - m_quickStopAlpha) * m_quickStopAccumulator
                + m_quickStopAlpha * curve * 2;
          }
          overPower = true;
          angularPower = -curve;
        } else {
          overPower = false;
          angularPower = throttle * curve - m_quickStopAccumulator;
    
          if (m_quickStopAccumulator > 1) {
            m_quickStopAccumulator -= 1;
          } else if (m_quickStopAccumulator < -1) {
            m_quickStopAccumulator += 1;
          } else {
            m_quickStopAccumulator = 0.0;
          }
        }
    
        double left = throttle + angularPower;
        double right = throttle - angularPower;
    
        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
          if (left > 1.0) {
            right -= left - 1.0;
            left = 1.0;
          } else if (right > 1.0) {
            left -= right - 1.0;
            right = 1.0;
          } else if (left < -1.0) {
            right -= left + 1.0;
            left = -1.0;
          } else if (right < -1.0) {
            left -= right + 1.0;
            right = -1.0;
          }
        }
        
        return new DriveSignal(left, right);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value    value to clip
     * @param deadband range around zero
     */
    protected double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
        } else {
            return 0.0;
        }
    }

    public class DriveSignal{
        public final double left, right;
        public DriveSignal(double left, double right){
            this.left = left;
            this.right = right;
        }
    }
}