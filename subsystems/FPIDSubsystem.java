package org.frc5587.lib.subsystems;

import org.frc5587.lib.pid.PID;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SpeedController;

public abstract class FPIDSubsystem extends PIDSubsystem {
    protected FPIDConstants constants;
    protected SpeedController[] motors;
    protected SpeedControllerGroup motorGroup;

    private final DigitalInput limitSwitch;

    // what type of value we can get from an encoder
    public static enum EncoderValueType {
        Velocity, Position
    }

    public static class FeedforwardModel extends ArmFeedforward {
        public final double kS;
        public final double kCos;
        public final double kV;
        public final double kA;

        // sets all given values for static, gravity, velocity, and acceleration gains.
        public FeedforwardModel(double kS, double kCos, double kV, double kA) {
            super(kS, kCos, kV, kA);
            this.kS = kS;
            this.kCos = kCos;
            this.kV = kV;
            this.kA = kA;
        }

        // use this for an elevator, or any FPIDSubsystem that does not require an angle gain.
        public FeedforwardModel(double kS, double kV, double kA) {
            this(kS, 0, kV, kA);
        }

        public double calculateNoAngle(double velocityRadPerSec, double accelRadPerSecSquared) {
            return (
                kS * Math.signum(velocityRadPerSec)
                + kV * velocityRadPerSec
                + kA * accelRadPerSecSquared
            );
        }
    }

    public static class FPIDConstants {
        public double speedMultiplier, gearing;
        public int limitSwitchPort, encoderCPR;
        public boolean limitSwitchInverted;
        public PID pid;
        public FeedforwardModel ff;

        public FPIDConstants(double speedMultiplier, double gearing, int encoderCPR, boolean limitSwitchInverted, int limitSwitchPort, PID pid, FeedforwardModel ff) {
            this.speedMultiplier = speedMultiplier;
            this.gearing = gearing;
            this.encoderCPR = encoderCPR;
            this.limitSwitchPort = limitSwitchPort;
            this.limitSwitchInverted = limitSwitchInverted;
            this.pid = pid;
            this.ff = ff;
        }
    }

    // pass motors as an array of SpeedControllers, eg. WPI_TalonFX[] or CANSparkMax[]
    public FPIDSubsystem(FPIDConstants constants, SpeedController[] motors) {
        super(new PIDController(constants.pid.kP, constants.pid.kI, constants.pid.kD));
        //enable PID control when starting

        this.enable();

        this.constants = constants;
        limitSwitch = new DigitalInput(constants.limitSwitchPort);

        motorGroup = new SpeedControllerGroup(motors);

        configureMotors();
    }

    // have the implementing class configure the motors
    public abstract void configureMotors();

    // the implementing class also needs to get and set encoder values
    // param type: what type of value we're getting from the encoder. values Position and Velocity as defined above in EncoderValueType
    public abstract double getEncoderValue(EncoderValueType type);

    public abstract void setEncoderPosition(double position);

    // move the mechanism based on a given throttle 
    public void moveByThrottle(double throttle) {
        motorGroup.set(-throttle * constants.speedMultiplier); // negative throttle is on purpose!
    }

    // move the mechanism based on a constant multiplier (for operation with buttons)
    public void moveByFixedSpeed(boolean inverted) {
        motorGroup.set((inverted ? -1 : 1) * constants.speedMultiplier);
    }

    // moves the mechanism based on voltage instead of speed
    public void moveByVolts(double voltage, boolean inverted) {
        motorGroup.setVoltage((inverted ? -1 : 1) * voltage);
    }

    // calculates Feedforward using given position and velocity values (using an ArmFeedforward)
    // override this method if using a subsystem other than an arm.
    public double calcFeedForward(double position, double velocity) {
        return constants.ff.calculate(position, velocity);
    }

    // calculates the Feedforward with a default value (using an ArmFeedforward)
    // override this method if using a subsystem other than an arm.
    public double calcFeedForward() {
        return constants.ff.calculate(Math.toRadians(SmartDashboard.getNumber("Goto Position", 0)), 0);
    }

    public void startPID() {
        SmartDashboard.putNumber("Goto Position", 0);
    }

    // displays various PID values on SmartDashboard
    // defaults to values from an arm. override this method if using a subsystem other than an arm.
    public void refreshPID() {
        SmartDashboard.putNumber("Angle", getPositionDegrees());
        SmartDashboard.putNumber("Encoder Val", getPositionRotation());
        SmartDashboard.putNumber("FF", calcFeedForward());
    }
    
    // gets the encoder's position in full arm rotations. 
    public double getPositionRotation() {
        return getEncoderValue(EncoderValueType.Position) / constants.gearing / constants.encoderCPR;
    }

    // returns the encoder position in degrees (based on the rotation)
    public double getPositionDegrees() {
        return getPositionRotation() * 360;
    }

    public DigitalInput getLimitSwitch() {
        return limitSwitch;
    }

    public boolean getLimitSwitchValue() {
        // if constants tells us to invert the limit switch, return it as a negated version
        return (constants.limitSwitchInverted ? !limitSwitch.get() : limitSwitch.get());
    }
    
    // uses PID output to move the mechanism
    @Override
    protected void useOutput(double output, double setpoint) {
        try {
            moveByThrottle(output);
        }
        catch(NullPointerException e) {
            System.out.println("NullPointerException " + e + " from useOutput. \n A constant was likely not given by DriveConstants object");
        }
    }

    // disables the subsystem without useing useOutput
    @Override
    public void disable() {
        this.m_enabled = false;
        motorGroup.set(0);
    }

    @Override
    protected double getMeasurement() {
        return getPositionDegrees();
    }

    @Override
    public void periodic() {
        refreshPID();
        // calculate a setpoint from the Feedforward model
        double setpoint = constants.ff.calculate(Math.toRadians(getPositionDegrees()), getEncoderValue(EncoderValueType.Velocity));
        // set the PIDController's setpoint to the above variable so it will automatically be used for output
        setSetpoint(setpoint);
        double output = getController().calculate(Math.toRadians(getPositionDegrees()));
        useOutput(output, setpoint);
    }

    // sets encoders back to 0
    public void resetEncoders() {
        setEncoderPosition(0);
    }

    // stops the speedcontroller group
    public void stop() {
        motorGroup.set(0);
    }

    // stops every motor without going through the speedcontroller group
    public void stopMotors() {
        for(SpeedController motor : motors) {
            motor.set(0);
        }
    }
}