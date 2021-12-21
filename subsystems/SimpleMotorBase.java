package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SimpleMotorBase extends SubsystemBase {
    protected SpeedControllerGroup motorGroup;
    protected double forwardThrottle;
    protected double backwardThrottle;

    public SimpleMotorBase(SpeedController[] motors, double forwardThrottle, double backwardThrottle) {
        // Create SpeedControllerGroup with the motors passed in as a parameter
        motorGroup = new SpeedControllerGroup(motors);
        this.forwardThrottle = forwardThrottle;
        this.backwardThrottle = backwardThrottle;
        configureMotors();
    }

    public abstract void configureMotors();

    public void forward() {
        motorGroup.set(forwardThrottle);
    }

    public void backward() {
        motorGroup.set(-backwardThrottle);
    }

    public void stop() {
        motorGroup.set(0);
    }
}
