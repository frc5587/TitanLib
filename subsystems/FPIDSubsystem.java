package org.frc5587.lib.subsystems;

import org.frc5587.lib.controllers.FFController;
import org.frc5587.lib.pid.PID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public abstract class FPIDSubsystem extends ProfiledPIDSubsystem {
    protected FPIDConstants constants;
    protected SpeedController[] motors;
    protected SpeedControllerGroup motorGroup;

    public static class FPIDConstants {
        public double speedMultiplier, gearing;
        public int encoderCPR, zeroOffset, switchPort;
        public boolean switchInverted;
        public PID pid;
        public FFController ff;
        public TrapezoidProfile.Constraints constraints;

        public FPIDConstants(
        double speedMultiplier, 
        double gearing, 
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
    * pass motors as a SpeedController, so they can be passed as one motor,
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
        
        this.enable();

        this.constants = constants;
        this.motorGroup = new SpeedControllerGroup(motorGroup);

        configureMotors();
    }

    /**
    * have the implementing class configure the motors 
    */
    public abstract void configureMotors();

    /**
    * the implementing class also needs to get and set encoder values
    * @param type what type of value we're getting from the encoder. values Position and Velocity as defined above in EncoderValueType
    */
    public abstract double getEncoderPosition();
    public abstract double getEncoderVelocity();
    public abstract void setEncoderPosition(double position);
    public abstract double rotationsToMeasurement(double rotations);

    @Override
    public abstract void periodic();

    /**
    * move the mechanism based on a given percent output (-1 to 1)
    */
    public void set(double throttle) {
        motorGroup.set(throttle);
        System.out.println("set the throttle to " + throttle);
    }

    /**
    * moves the mechanism based on voltage instead of speed
    */
    public void setVoltage(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    @Override
    public double getMeasurement() {
        return rotationsToMeasurement(getRotations());
    }

    /**
    * uses PID output to move the mechanism
    */
    @Override
    protected abstract void useOutput(double output, TrapezoidProfile.State profileState);

    /** 
    * disables PID without using useOutput
    */
    @Override
    public void disable() {
        this.m_enabled = false;
        try {
            motorGroup.set(0);
        }
        catch(NullPointerException e) {
            System.out.println(e);
        }
    }

    /**
    * divides @param value by the gearing set in constants
    */
    public double applyGearing(double value) {
        return value / constants.gearing;
    }

    /**
    * divides @param value by the encoder counts per revolution set in constants
    */
    public double applyCPR(double value) {
        return value / constants.encoderCPR;
    }

    /**
    * gets the rotations of the subsystem, accounting for encoderCPR and gearing.
    */
    public double getRotations() {
        return applyCPR(applyGearing(getEncoderPosition()));
    }

    /**
    * gets the velocity of the subsystem (in RPS), accounting for encoderCPR and gearing.
    */
    public double getRotationsPerSecond() {
        return applyCPR(applyGearing(getEncoderVelocity()));
    }

    /** 
    * sets encoders back to 0
    */
    public void resetEncoders() {
        setEncoderPosition(constants.zeroOffset);
    }

    /**
    * stops the speedcontroller group
    */
    public void stop() {
        motorGroup.set(0);
    }
}
