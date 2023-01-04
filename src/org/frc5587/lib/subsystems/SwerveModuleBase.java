package org.frc5587.lib.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;

public abstract class SwerveModuleBase extends ProfiledPIDSubsystem {
    protected MotorController driveMotor;
    protected MotorController turnMotor;
    protected SwerveModuleConstants constants;
    
    public static class SwerveModuleConstants {
        public final double gearing;
        public final int encoderCPR;
        public final ProfiledPIDController pid;

        public SwerveModuleConstants(double gearing, int encoderCPR, ProfiledPIDController pid) {
            this.gearing = gearing;
            this.encoderCPR = encoderCPR;
            this.pid = pid;
        }
    }

    public SwerveModuleBase(MotorController driveMotor, MotorController turnMotor, SwerveModuleConstants constants) {
        super(constants.pid);

        this.driveMotor = driveMotor;
        this.turnMotor = turnMotor;
        this.constants = constants;

        configureMotors();
        this.enable();
    }

    /** @return Raw encoder ticks from the rotational motor for angle */
    public abstract double getRotationPositionTicks();

    /** @return Raw encoder ticks from the drive motor position */
    public abstract double getDrivePositionTicks();

    /** @return Raw encoder ticks per second from rotational motor for velocity */
    public abstract double getRotationVelocityTicksPerSecond();

    /** @return Raw encoder ticks per second from drive motor for velocity */
    public abstract double getDriveVelocityTicksPerSecond();

    /** @param position the position to set the encoder to */
    public abstract void setRotationEncoderPosition(double position);

    /** @param position the position to set the encoder to */
    public abstract void setDriveEncoderPosition(double position);

    /**
     * Configure the motors. This should handle things like reversal,
     * idle mode, motor sensor ports, etc.
     */
    protected abstract void configureMotors();

    /**
     * @param value value to divide by the gearing set in constants
     */
    private double applyGearing(double value) {
        return value / constants.gearing;
    }

    /**
     * @param value value to divide by the encoder counts per revolution set in constants
     */
    private double applyCPR(double value) {
        return value / constants.encoderCPR;
    }

    /**
     * @return the position of the subsystem in rotations,
     *         accounting for gearing and encoder counts per revolution.
     */
    public double getRotations() {
        return applyCPR(applyGearing(getRotationPositionTicks()));
    }

    /**
     * Sets this module's rotation position in radians
     * @param angleRadians
     */
    public void setAngle(double angleRadians) {
        setGoal(angleRadians);
    }

    /**
     * Sets this module's rotation position in degrees
     * @param angleDegrees
     */
    public void setAngleDegrees(double angleDegrees) {
        setGoal((angleDegrees * Math.PI) / 180);
    }

    /**
     * Sets motor speed in percent output
     * @param speed a double -1 to 1
     */
    public void setSpeed(double speed) {
        driveMotor.set(speed);
    }

    /**
     * Sets motor output in voltage
     * @param voltage a double -12 to 12 or between min/max voltage the motor can handle
     */
    public void setVoltage(double voltage) {
        driveMotor.setVoltage(voltage);
    }

    /**
     * Moves both motors of the swerve module at once
     * @param speed the motor's speed in percent output, a double -1 to 1
     * @param angle the module's angle in radians
     */
    public void drive(double speed, double angle) {
        setSpeed(speed);
        setAngle(angle);
    }

    /**
     * Moves both motors of the swerve module at once
     * @param voltage the motor's output in voltage, a double -12 to 12 or between min/max voltage the motor can handle
     * @param angle the module's angle in radians
     */
    public void driveVoltage(double voltage, double angle) {
        setVoltage(voltage);
        setAngle(angle);
    }

    /**
     * Moves both motors of the swerve module at once
     * @param voltage the motor's output in voltage, a double -12 to 12 or between min/max voltage the motor can handle
     * @param angleDegrees the module's angle in radians
     */
    public void driveVoltageDegrees(double voltage, double angleDegrees) {
        setVoltage(voltage);
        setAngleDegrees(angleDegrees);
    }

    /**
     * Sets motor speed in percent output
     * @param speed a double -1 to 1
     */
    public void rotateMotor(double speed) {
        turnMotor.set(speed);
    }

    /**
     * Sets motor output in voltage
     * @param voltage a double -12 to 12 or between min/max voltage the motor can handle
     */
    public void rotateMotorVolts(double voltage) {
        turnMotor.setVoltage(voltage);
    }

    @Override
    public double getMeasurement() {
        return getRotations() * 2 * Math.PI;
    }
    
    @Override
    public void useOutput(double output, TrapezoidProfile.State state) {
        rotateMotorVolts(output);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}
