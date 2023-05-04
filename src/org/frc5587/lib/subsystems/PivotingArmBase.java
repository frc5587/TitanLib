package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class PivotingArmBase extends ProfiledPIDSubsystem {
    protected ProfiledPIDController pidController;
    protected ArmFeedforward ffController;
    protected PivotingArmConstants constants;
    protected MotorController motor;
    protected String subsystemName;

    public static class PivotingArmConstants {
        public final double gearing, velocityDenominator, offsetFromHorizontalRadians;
        public final double[] softLimits;
        public final int zeroOffset, encoderCPR;
        public final ProfiledPIDController pid;
        public final ArmFeedforward ff;

        public PivotingArmConstants(
                double gearing,
                double velocityDenominator,
                double offsetFromHorizontalRadians,
                double[] softLimits,
                int zeroOffset,
                int encoderCPR,
                ProfiledPIDController pid,
                ArmFeedforward ff) {
            this.gearing = gearing;
            this.velocityDenominator = velocityDenominator;
            this.offsetFromHorizontalRadians = offsetFromHorizontalRadians;
            this.softLimits = softLimits;
            this.zeroOffset = zeroOffset;
            this.encoderCPR = encoderCPR;
            this.pid = pid;
            this.ff = ff;
        }
    }
    
    public PivotingArmBase(String subsystemName, PivotingArmConstants constants, MotorController motor) {
        super(constants.pid);
        this.constants = constants;
        this.motor = motor;
        this.pidController = getController();
        this.subsystemName = subsystemName;
        this.ffController = constants.ff;
        
        SmartDashboard.putBoolean(subsystemName + " Output On?", true);
    }

    /**
     * Sets the motors to a percent output
     * 
     * @param value the percent output to set the motors to (a double -1 to 1)
     */
    public void set(double throttle) {
        motor.set(throttle);
    }

    /**
     * Moves the motors by voltage instead of percent output
     * 
     * @param voltage the voltage to set the motors to (a double -12 to 12)
     */
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    /* CALCULATIONS AND UTIL */
    /* The implementing class needs to handle encoders for us as we don't know what type the arm is using */
    /** @return the encoder's position */
    public abstract double getEncoderPosition();

    /** @return the encoder's velocity */
    public abstract double getEncoderVelocity();

    /** @param position the position to set the encoder to */
    public abstract void setEncoderPosition(double position);

    /**
     * Configure the motors. This should handle things like reversal,
     * idle mode, motor sensor ports, etc.
     */
    public abstract void configureMotors();

    /**
     * @param value value to divide by the gearing set in constants
     */
    public double applyGearing(double value) {
        return value / constants.gearing;
    }

    /**
     * @param value value to divide by the encoder counts per revolution set in constants
     */
    public double applyCPR(double value) {
        return value / constants.encoderCPR;
    }

    /**
     * @return the position of the subsystem in rotations,
     *         accounting for gearing and encoder counts per revolution.
     */
    public double getRotations() {
        return applyCPR(applyGearing(getEncoderPosition()));
    }

    /**
     * @return the velocity of the subsystem (in RPS),
     *         accounting for encoderCPR and gearing.
     */
    public double getRotationsPerSecond() {
        return applyCPR(applyGearing(getEncoderVelocity()));
    }

    /**
     * @return The arm's angle in radians
     */
    @Override
    public double getMeasurement() {
        return getRotations() * 2 * Math.PI;
    }

    /**
    * @return the angle of the arm in degrees
    */
    public double getAngleDegrees() {
        return getRotations() * 360;
    }

    /**
    * @return the angle of the arm in radians
    */
    public double getAngleRadians() {
        return Units.degreesToRadians(getAngleDegrees());
    }

    /**
     * @param ticks the number of encoder ticks to convert to radians
     * @return the number of radians corresponding to the encoder ticks
     */
    public double encoderTicksToRadians(double ticks) {
        return applyCPR(applyGearing(ticks)) * 2 * Math.PI;
    }

    /**
     * @param radians the number of radians to convert to encoder ticks
     * @return the number of encoder ticks corresponding to radians
     */
    public int radiansToEncoderTicks(double radians) {
        return Math.round((float) ((radians * constants.gearing * constants.encoderCPR) / 2 / Math.PI));
    }

    /**
     * Sets encoders back to the zero offset of the subsystem.
     */
    public void resetEncoders() {
        setEncoderPosition(constants.zeroOffset);
    }

    /**
    * Stops the MotorControllerGroup
    */
    public void stop() {
        motor.set(0);
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double ff = ffController.calculate(setpoint.position+constants.offsetFromHorizontalRadians, setpoint.velocity);
        
        /** if the driver has set output on, useOutput. */
        if(SmartDashboard.getBoolean(subsystemName + " Output On?", true)) {
            setVoltage(output + ff);
        }
        /** otherwise, set output to 0 */
        else {
            setVoltage(0);
        }
    }
}