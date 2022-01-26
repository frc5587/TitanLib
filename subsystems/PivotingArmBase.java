package org.frc5587.lib.subsystems;

import org.frc5587.lib.controllers.FFController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class PivotingArmBase extends FPIDSubsystem {
    protected FFController ffController;
    protected ProfiledPIDController pidController;
    protected DigitalInput limitSwitch;
    
    public PivotingArmBase(FPIDConstants constants, MotorController motorGroup) {
        super(constants, motorGroup);
        ffController = constants.ff;
        pidController = getController();
        limitSwitch = new DigitalInput(constants.switchPort);
    }

    /**
    * @return the limit switch's state, inverted if necessary.
    */
    public boolean getLimitSwitchValue() {
        return (constants.switchInverted ? !limitSwitch.get() : limitSwitch.get());
    }

    @Override
    public double rotationsToMeasurement(double rotations) {
        return rotations * 2 * Math.PI;
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

    @Override
    public void periodic() {
        TrapezoidProfile.State goalState = pidController.getGoal();
        /** calculate Feedforward and PID with position and velocity from the pidController */
        double ff = ffController.calculateArm(goalState.position, goalState.velocity);
        double output = pidController.calculate(getMeasurement(), goalState.position);
        /** print various values to SmartDashboard */
        SmartDashboard.putNumber("Angle in Radians", getMeasurement());
        SmartDashboard.putNumber("Angle in Degrees", getAngleDegrees());
        SmartDashboard.putNumber("FeedForward", ff);
        SmartDashboard.putNumber("Output calculated", output);
        SmartDashboard.putNumber("Output passed", output + ff);
        SmartDashboard.putNumber("Goal", goalState.position);
        Shuffleboard.update();
        /** if the driver has set output on, useOutput. */
        if(SmartDashboard.getBoolean("OUTPUT ON?", true)) {
            /** output should be feedforward + calculated PID. */
            useOutput(ff + output, pidController.getGoal());
        }
        /** otherwise, set output to 0 */
        else {
            useOutput(0, new TrapezoidProfile.State());
        }
    }

    @Override
    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        SmartDashboard.putNumber("OUTPUT USED", output);
        SmartDashboard.putNumber("SETPOINT USED", setpoint.position);
        SmartDashboard.putNumber("GOAL USED", pidController.getGoal().position);
        SmartDashboard.putBoolean("AT SETPOINT", pidController.atGoal());

        /** SOFT LIMITS */
        /** if the limit switch is pressed and the arm is powered to move downward, set the voltage to 0 */
        if(getLimitSwitchValue() && output < 0) {
            setVoltage(0);
        }

        /** if the arm is above the limit and is powered to move upward, set the voltage to 0 */
        else if(!getLimitSwitchValue() && getMeasurement() > constants.softLimits[1] && output > 0) {
            setVoltage(0);
        }

        else {
            setVoltage(output);
        }
    }
}
