package org.frc5587.lib.subsystems;

import org.frc5587.lib.controllers.FFController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class ElevatorBase extends FPIDSubsystem {
    protected FPIDConstants constants;
    protected MotorController motor;
    protected FFController ffController;
    protected ProfiledPIDController pidController;
    
    public ElevatorBase(FPIDConstants constants, MotorController motor) {
        super(constants, motor);
        this.constants = constants;
        this.motor = motor;
        this.ffController = constants.ff;
    }

    @Override
    public double rotationsToMeasurement(double rotations) {
        return rotations * constants.universalConversionFactor;
    }

    @Override
    public void periodic() {
        TrapezoidProfile.State goalState = pidController.getGoal();
        /** calculate Feedforward and PID with position and velocity from the pidController */
        double ff = ffController.calculateElevator(goalState.velocity);
        double output = pidController.calculate(getMeasurement(), goalState.position);
        /** print various values to SmartDashboard */
        SmartDashboard.putNumber("Height in Meters", getMeasurement());
        SmartDashboard.putNumber("Rotations", getRotations());
        SmartDashboard.putNumber("FeedForward", ff);
        SmartDashboard.putNumber("Output calculated", output);
        SmartDashboard.putNumber("Output passed", output + ff);
        SmartDashboard.putNumber("Goal", goalState.position);
        Shuffleboard.update();

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

        if(SmartDashboard.getBoolean("OUTPUT ON?", true)) {
            setVoltage(output);
        }
        else if(!SmartDashboard.getBoolean("OUTPUT ON?", true) && getMeasurement() > constants.softLimits[1] && output > 0) {
            setVoltage(0);
        }
        else {
            useOutput(0, new TrapezoidProfile.State());
        }
    }
}
