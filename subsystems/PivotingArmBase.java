package org.frc5587.lib.subsystems;

import org.frc5587.lib.controllers.FFController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public abstract class PivotingArmBase extends FPIDSubsystem {
    protected ProfiledPIDController pidController;
    protected FFController ffController;
    protected DigitalInput limitSwitch;
    protected PivotingArmConstants armConstants;

    public static class PivotingArmConstants {
        public double[] softLimits;
        public int switchPort;
        public boolean switchInverted;

        public PivotingArmConstants(double[] softLimits, int switchPort, boolean switchInverted) {
            this.softLimits = softLimits;
            this.switchPort = switchPort;
            this.switchInverted = switchInverted;
        }
    }
    
    public PivotingArmBase(FPIDConstants constants, MotorController motor) {
        super(constants, motor);
        ffController = constants.ff;
        pidController = getController();
        limitSwitch = new DigitalInput(armConstants.switchPort);
    }

    /**
    * @return the limit switch's state, inverted if necessary.
    */
    public boolean getLimitSwitchValue() {
        return (armConstants.switchInverted ? !limitSwitch.get() : limitSwitch.get());
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

    public double calcOutput(TrapezoidProfile.State state) {
        return pidController.calculate(getMeasurement(), state.position);
    }

    public double calcFF(TrapezoidProfile.State state) {
        return ffController.calculateArm(getMeasurement(), state.velocity);
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
        else if(!getLimitSwitchValue() && getMeasurement() > armConstants.softLimits[1] && output > 0) {
            setVoltage(0);
        }

        else {
            setVoltage(output);
        }
    }
}
