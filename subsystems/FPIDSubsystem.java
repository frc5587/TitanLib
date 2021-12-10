package org.frc5587.lib.subsystems;

import org.frc5587.lib.controllers.FFPIDController;
import org.frc5587.lib.pid.PID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public abstract class FPIDSubsystem extends PIDSubsystem {
    protected FPIDConstants constants;
    protected SpeedController[] motors;
    protected SpeedControllerGroup motorGroup;

    // what type of value we can get from an encoder
    public static enum EncoderValueType {
        Velocity, Position
    }

    public static class FPIDConstants {
        public double speedMultiplier, gearing;
        public int encoderCPR;
        public PID pid;
        public FFPIDController ff;

        public FPIDConstants(double speedMultiplier, double gearing, int encoderCPR, PID pid, FFPIDController ff) {
            this.speedMultiplier = speedMultiplier;
            this.gearing = gearing;
            this.encoderCPR = encoderCPR;
            this.pid = pid;
            this.ff = ff;
        }
    }

    /** 
    * pass motors as a SpeedControllerGroup 
    */
    public FPIDSubsystem(FPIDConstants constants, SpeedControllerGroup motorGroup) {
        super(new PIDController(constants.pid.kP, constants.pid.kI, constants.pid.kD));
        /**
        * enable PID control when starting
        */
        this.enable();

        this.constants = constants;
        this.motorGroup = motorGroup;

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
    public abstract double getEncoderValue(EncoderValueType type);
    public abstract void setEncoderPosition(double position);

    /**
    * the implementing class should decide how to calculate Feedforward 
    * because we don't know what model to use
    */
    public abstract double calcFeedForward(double position, double velocity);
    public abstract double calcFeedForward();

    /**
    * it should also do getMeasurement because we don't know what measurement to use
    * (measurement could be an angle, height, etc)
    */
    @Override
    public abstract double getMeasurement();

    @Override
    public abstract void periodic();

    /**
    * move the mechanism based on a given throttle 
    */
    public void moveByThrottle(double throttle) {
        motorGroup.set(throttle * constants.speedMultiplier);
    }

    /**
    *  move the mechanism based on a constant multiplier (for operation with buttons)
    */
     public void moveByFixedSpeed() {
        motorGroup.set(constants.speedMultiplier);
    }

    public void moveFixedReversed() {
        motorGroup.set(-constants.speedMultiplier);
    }

    /**
    * moves the mechanism based on voltage instead of speed
    */
    public void moveByVolts(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    /**
    * uses PID output to move the mechanism
    */
    @Override
    protected void useOutput(double output, double setpoint) {
        try {
            moveByThrottle(output);
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException " + e + " from useOutput. \n A constant was likely not given by DriveConstants object");
        }
    }

    /** 
    * disables the subsystem without using useOutput
    */
    @Override
    public void disable() {
        this.m_enabled = false;
        motorGroup.set(0);
    }

    /** 
    * sets encoders back to 0
    */
    public void resetEncoders() {
        setEncoderPosition(0);
    }

    /**
    * stops the speedcontroller group
    */
    public void stop() {
        motorGroup.set(0);
    }
}
