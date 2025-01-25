package org.frc5587.lib.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public abstract class TalonPivotingArmBase extends ProfiledPIDSubsystem {
    private final ArmFeedforward ffController;
    private final PivotingArmConstants constants;
    private final TalonFX motor;

    public static class PivotingArmConstants {
        public final double gearing;
        public final Rotation2d[] softLimits;
        public final Rotation2d offsetFromHorizontal, zeroOffset;
        public final ProfiledPIDController pid;
        public final ArmFeedforward ff;

        public PivotingArmConstants(
                double gearing,
                Rotation2d offsetFromHorizontal,
                Rotation2d[] softLimits,
                Rotation2d zeroOffset,
                ProfiledPIDController pid,
                ArmFeedforward ff) {
            this.gearing = gearing;
            this.softLimits = softLimits;
            this.offsetFromHorizontal = offsetFromHorizontal;
            this.zeroOffset = zeroOffset;
            this.pid = pid;
            this.ff = ff;
        }
    }
    
    public TalonPivotingArmBase(PivotingArmConstants constants, TalonFX motor) {
        super(constants.pid);
        this.constants = constants;
        this.motor = motor;
        this.ffController = constants.ff;
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
    public abstract Rotation2d getEncoderPosition();

    /** @return the encoder's velocity */
    public abstract double getEncoderVelocity();

    /** @param position the position to set the encoder to */
    public abstract void setEncoderPosition(Rotation2d position);

    /**
     * Configure the motors. This should handle things like reversal,
     * idle mode, motor sensor ports, etc.
     * 
     * Note: You MUST use this method in the subclass constructor, otherwise
     * your motors will not be configured. This cannot be implemented in the
     * PivotingArmBase class constructor because of NullPointer/initialization
     * order.
     */
    public abstract void configureMotors();

    /**
     * @param value value to divide by the gearing set in constants
     */
    public double applyGearing(double value) {
        return value / (constants.gearing);
    }

    /**
     * @return the position of the subsystem in rotations,
     *         accounting for gearing and encoder counts per revolution.
     */
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(applyGearing(getEncoderPosition().getRotations()));
    }

    /**
     * @return the velocity of the subsystem (in Rotations Per Second),
     *         accounting for gearing.
     */
    public double getVelocityRotationsPerSecond() {
        return applyGearing(getEncoderVelocity());
    }

    /**
     * @return The arm's angle in radians
     */
    @Override
    public double getMeasurement() {
        return getAngleRadians();
    }

    /**
    * @return the angle of the arm in degrees
    */
    public double getAngleDegrees() {
        return getPosition().getDegrees();
    }

    /**
    * @return the angle of the arm in radians
    */
    public double getAngleRadians() {
        return getPosition().getRadians();
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
        double ff = ffController.calculate(setpoint.position+constants.offsetFromHorizontal.getRadians(), setpoint.velocity);
        setVoltage(output + ff);
    }
}