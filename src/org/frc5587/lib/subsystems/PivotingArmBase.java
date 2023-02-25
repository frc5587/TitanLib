package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.ArrayList;
import java.util.Hashtable;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class PivotingArmBase extends ProfiledPIDSubsystem {
    protected ProfiledPIDController pidController;
    protected ArmFeedforward ffController;
    protected PivotingArmConstants constants;
    protected MotorController motor;
    /** 
     * How to lookup from this table: 
     * <ul>
     * <li> Key: int switchPort </li>
     * <li> Value: {DigitalInput limitSwitch, boolean inverted} </li>
     * </ul>
     */
    protected Hashtable<Integer, ArrayList<Object>> switchTable = new Hashtable<Integer, ArrayList<Object>>();

    public static class PivotingArmConstants {
        public final double gearing;
        public final double[] softLimits;
        public final int zeroOffset, encoderCPR;
        public final int[] switchPorts;
        public final boolean[] switchesInverted;
        public final ProfiledPIDController pid;
        public final ArmFeedforward ff;

        public PivotingArmConstants(
                double gearing,
                double[] softLimits,
                int zeroOffset,
                int encoderCPR,
                int[] switchPorts,
                boolean[] switchesInverted,
                ProfiledPIDController pid,
                ArmFeedforward ff) {
            this.gearing = gearing;
            this.softLimits = softLimits;
            this.zeroOffset = zeroOffset;
            this.encoderCPR = encoderCPR;
            this.switchPorts = switchPorts;
            this.switchesInverted = switchesInverted;
            this.pid = pid;
            this.ff = ff;
        }
    }
    
    public PivotingArmBase(PivotingArmConstants constants, MotorController motor) {
        super(constants.pid);
        this.constants = constants;
        this.motor = motor;
        this.pidController = getController();
        ffController = constants.ff;
        for(int i = 0; i < constants.switchPorts.length; i++) {
            ArrayList<Object> values = new ArrayList<Object>();
            values.add(new DigitalInput(constants.switchPorts[i]));
            values.add(constants.switchesInverted[i]);

            /** 
             * Lookup from this table: 
             * Key: int switchPort
             * Value: {DigitalInput limitSwitch, boolean inverted}
             */
            switchTable.put(constants.switchPorts[i], values);
        }
        
        SmartDashboard.getBoolean("ARM OUTPUT ON?", true);
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
    /**
     * @param switchPort the port of the limit switch we want the value of
     * @return the limit switch's state, inverted if necessary.
     */
    public boolean getLimitSwitchValue(int switchPort) {
        DigitalInput lSwitch = (DigitalInput) switchTable.get(switchPort).get(0);
        return ((boolean) switchTable.get(switchPort).get(1) ? !lSwitch.get() : lSwitch.get());
    }

    /**
     * @param switchPort the port of the limit switch you want to get
     * @return the DigitalInput of the switch
     */
    public DigitalInput getLimitSwitchObject(int switchPort) {
        return (DigitalInput) switchTable.get(switchPort).get(0);
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
        return getRotations() * 2 * Math.PI;
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
        double ff = ffController.calculate(setpoint.position, setpoint.velocity);
        //TODO: remove debug prints once we know this code works
        SmartDashboard.putNumber("ARM FEEDFORWARD", ff);
        SmartDashboard.putNumber("ARM OUTPUT USED", output);
        SmartDashboard.putNumber("ARM SETPOINT USED", setpoint.position);
        SmartDashboard.putNumber("ARM GOAL USED", pidController.getGoal().position);
        SmartDashboard.putBoolean("ARM AT SETPOINT", pidController.atGoal());

        /** SOFT LIMITS */
        /** if the driver has set output on, useOutput. */
        if(SmartDashboard.getBoolean("ARM OUTPUT ON?", true)) {
            /** output should be feedforward + calculated PID. */
            /** if the limit switch is pressed and the arm is powered to move downward, set the voltage to 0 */
            if(getLimitSwitchValue(constants.switchPorts[0]) && output < 0) {
                setVoltage(0);
            }

            /** if the arm is above the limit and is powered to move upward, set the voltage to 0 */
            else if(!getLimitSwitchValue(constants.switchPorts[0]) && getMeasurement() > constants.softLimits[1] && output > 0) {
                setVoltage(0);
            }

            else {
                setVoltage(output + ff);
            }
        }
        /** otherwise, set output to 0 */
        else {
            setVoltage(0);
        }

        
    }
}
