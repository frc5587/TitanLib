package org.frc5587.lib.subsystems;

import org.frc5587.lib.controllers.FFController;
import org.frc5587.lib.pid.PID;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public abstract class FPIDSubsystem extends ProfiledPIDSubsystem {
    protected FPIDConstants constants;
    protected SpeedController[] motors;
    protected SpeedController motorGroup;

    public static class FPIDConstants {
        public double speedMultiplier, gearing;
        public double[] softLimits;
        public int encoderCPR, zeroOffset, switchPort;
        public boolean switchInverted;
        public PID pid;
        public FFController ff;
        public TrapezoidProfile.Constraints constraints;

        public FPIDConstants(
        double speedMultiplier, 
        double gearing, 
        double[] softLimits,
        int zeroOffset, 
        int encoderCPR, 
        int switchPort, 
        boolean switchInverted, 
        PID pid, 
        FFController ff, 
        TrapezoidProfile.Constraints constraints
        ) 
        {
            this.speedMultiplier = speedMultiplier;
            this.gearing = gearing;
            this.softLimits = softLimits;
            this.zeroOffset = zeroOffset;
            this.encoderCPR = encoderCPR;
            this.switchPort = switchPort;
            this.switchInverted = switchInverted;
            this.pid = pid;
            this.ff = ff;
            this.constraints = constraints;
        }
    }

    /** 
    * Pass motors as a SpeedController, so they can be passed as one motor,
    * or made into a SpeedControllerGroup if there are multiple.
    */
    public FPIDSubsystem(FPIDConstants constants, SpeedController motorGroup) {
        super(
            new ProfiledPIDController(
                constants.pid.kP, 
                constants.pid.kI, 
                constants.pid.kD,
                constants.constraints
            )
        );
        
        // this.enable();
        this.constants = constants;
        this.motorGroup = motorGroup;

        configureMotors();
    }

    /**
    * Have the implementing class configure the motors 
    */
    public abstract void configureMotors();

    /**
    * the implementing class also needs to get and set encoder values, as we don't know the encoder type
    */
    /** @return the encoder's position */
    public abstract double getEncoderPosition();
    /** @return the encoder's velocity */
    public abstract double getEncoderVelocity();
    /** @param position the position to set the encoder to */
    public abstract void setEncoderPosition(double position);
    /**
     * @param rotations
     * @return the measurement used by the subsystem, converted from rotations
     */
    public abstract double rotationsToMeasurement(double rotations);

    @Override
    public abstract void periodic();

    /**
    * Sets the motors to a percent output
    * @param value the percent output to set the motors to (a double -1 to 1)
    */
    public void set(double throttle) {
        motorGroup.set(throttle);
        System.out.println("set the throttle to " + throttle);
    }

    /**
    * Moves the motors by voltage instead of percent output
    * @param voltage the voltage to set the motors to (a double -12 to 12)
    */
    public void setVoltage(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    /**
     * @return the subsystem's measurement in the units used by the implementing subsystem.
     */
    @Override
    public double getMeasurement() {
        return rotationsToMeasurement(getRotations());
    }

    /**
    * Uses PID output to move the mechanism
    * <p> Make sure that the implementing class overrides this, because all
    * FPIDSubsystems will use output differently.
    */
    @Override
    protected abstract void useOutput(double output, TrapezoidProfile.State profileState);

    /** 
    * Disables PID without using useOutput
    */
    @Override
    public void disable() {
        this.m_enabled = false;
        try {
            motorGroup.set(0);
        }
        catch(NullPointerException e) {
            System.out.println(e + " Could not get motor group.");
        }
    }

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
    * accounting for gearing and encoder counts per revolution.
    */
    public double getRotations() {
        return applyCPR(applyGearing(getEncoderPosition()));
    }

    /**
    * @return the velocity of the subsystem (in RPS),
    * accounting for encoderCPR and gearing.
    */
    public double getRotationsPerSecond() {
        return applyCPR(applyGearing(getEncoderVelocity()));
    }

    /** 
    * Sets encoders back to the zero offset of the subsystem.
    */
    public void resetEncoders() {
        setEncoderPosition(constants.zeroOffset);
    }

    /**
    * Stops the SpeedControllerGroup
    */
    public void stop() {
        motorGroup.set(0);
    }
}
