package org.frc5587.lib.subsystems;

import java.util.ArrayList;
import java.util.Hashtable;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public abstract class ElevatorBase extends ProfiledPIDSubsystem {
    protected ElevatorConstants constants;
    protected MotorController motor;
    protected ElevatorFeedforward ffController;
    protected ProfiledPIDController pidController;
    /** 
     * How to lookup from this table: 
     * <ul>
     * <li> Key: int switchPort </li>
     * <li> Value: {DigitalInput limitSwitch, boolean inverted} </li>
     * </ul>
     */
    protected Hashtable<Integer, ArrayList<Object>> switchTable = new Hashtable<Integer, ArrayList<Object>>();

    public static final class ElevatorConstants {
        public final double gearing, rotationsToMeters;
        public final double[] softLimits;
        public final int zeroOffset, encoderCPR;
        public final int[] switchPorts; 
        public final boolean[] switchesInverted;
        public final ProfiledPIDController pid;
        public final ElevatorFeedforward ff;

        public ElevatorConstants(
                double gearing, 
                double rotationsToMeters,
                double[] softLimits,
                int zeroOffset,
                int encoderCPR,
                int[] switchPorts,
                boolean[] switchesInverted,
                ProfiledPIDController pid,
                ElevatorFeedforward ff) {
            this.gearing = gearing;
            this.rotationsToMeters = rotationsToMeters;
            this.softLimits = softLimits;
            this.zeroOffset = zeroOffset;
            this.encoderCPR = encoderCPR;
            this.switchPorts = switchPorts;
            this.switchesInverted = switchesInverted;
            this.pid = pid;
            this.ff = ff;
        }
    }
    
    public ElevatorBase(ElevatorConstants constants, MotorController motor) {
        super(constants.pid);
        this.constants = constants;
        this.motor = motor;
        this.ffController = constants.ff;
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

        SmartDashboard.getBoolean("ELEVATOR OUTPUT ON?", true);
    }
    
    /**
     * Sets the motors to a percent output
     * @param value the percent output to set the motors to (a double -1 to 1)
     */
    public void set(double throttle) {
        motor.set(throttle);
    }

    /**
     * Moves the motors by voltage instead of percent output
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
    /* The implementing class needs to handle encoders for us as we don't know what type the elevator is using */
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
     * Converts rotations to the measurement used by the subsystem (meters in this case). 
     * This is useful for getting measurements like the position in meters, velocity in m/s, etc.
     * @param rotations the number of rotations to convert
     * @return the measurement used by the subsystem, converted from rotations.
     */
    public double rotationsToMeasurement(double rotations) {
        return rotations * constants.rotationsToMeters;
    }

    /**
     * @return The elevator's position in meters
     */
    @Override
    public double getMeasurement() {
        return rotationsToMeasurement(getRotations());
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
        SmartDashboard.putNumber("ELEVATOR FEEDFORWARD", ff);
        SmartDashboard.putNumber("ELEVATOR OUTPUT USED", output);
        SmartDashboard.putNumber("ELEVATOR SETPOINT USED", setpoint.position);
        SmartDashboard.putNumber("ELEVATOR GOAL USED", pidController.getGoal().position);
        SmartDashboard.putBoolean("ELEVATOR AT SETPOINT", pidController.atGoal());

        /** if the driver has set output on, useOutput. */
        if(SmartDashboard.getBoolean("ELEVATOR OUTPUT ON?", true)) {
            /** output should be feedforward + calculated PID. */
            /** if the limit switch is pressed and the elevator is powered to move downward, set the voltage to 0 */
            if(getLimitSwitchValue(constants.switchPorts[0]) && output < 0) {
                setVoltage(0);
            }

            /** if the elevator is above the limit and is powered to move upward, set the voltage to 0 */
            else if(!getLimitSwitchValue(constants.switchPorts[0]) && getMeasurement() > constants.softLimits[1] && output > 0) {
                setVoltage(0);
            }

            else {
                setVoltage(output + ff);
            }
        }
        else {
            setVoltage(0);
        }
    }
}
