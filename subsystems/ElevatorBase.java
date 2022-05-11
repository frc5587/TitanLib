package org.frc5587.lib.subsystems;

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
    protected DigitalInput limitSwitch;

    public static final class ElevatorConstants {
        public final double gearing, rotationsToMeters;
        public final double[] softLimits;
        public final int zeroOffset, encoderCPR, switchPort;
        public final boolean switchInverted;
        public final ProfiledPIDController pid;
        public final ElevatorFeedforward ff;

        public ElevatorConstants(
                double gearing, 
                double rotationsToMeters,
                double[] softLimits,
                int zeroOffset,
                int encoderCPR,
                int switchPort,
                boolean switchInverted,
                ProfiledPIDController pid,
                ElevatorFeedforward ff) {
            this.gearing = gearing;
            this.rotationsToMeters = rotationsToMeters;
            this.softLimits = softLimits;
            this.zeroOffset = zeroOffset;
            this.encoderCPR = encoderCPR;
            this.switchPort = switchPort;
            this.switchInverted = switchInverted;
            this.pid = pid;
            this.ff = ff;
        }
    }
    
    public ElevatorBase(ElevatorConstants constants, MotorController motor) {
        super(constants.pid);
        this.constants = constants;
        this.motor = motor;
        this.ffController = constants.ff;
        this.limitSwitch = new DigitalInput(constants.switchPort);
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
     * @return the limit switch's state, inverted if necessary.
     */
    public boolean getLimitSwitchValue() {
        return (constants.switchInverted ? !limitSwitch.get() : limitSwitch.get());
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
        SmartDashboard.putNumber("FEEDFORWARD", ff);
        SmartDashboard.putNumber("OUTPUT USED", output);
        SmartDashboard.putNumber("SETPOINT USED", setpoint.position);
        SmartDashboard.putNumber("GOAL USED", pidController.getGoal().position);
        SmartDashboard.putBoolean("AT SETPOINT", pidController.atGoal());

        if(SmartDashboard.getBoolean("OUTPUT ON?", true)) {
            setVoltage(output + ff);
        }
        else if(!SmartDashboard.getBoolean("OUTPUT ON?", true) && getMeasurement() > constants.softLimits[1] && output > 0) {
            setVoltage(0);
        }
        else {
            useOutput(0, new TrapezoidProfile.State());
        }
    }
}
