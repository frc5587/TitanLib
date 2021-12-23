package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SimpleMotorBase extends SubsystemBase {
    protected SpeedController motorController;
    protected double forwardThrottle;
    protected double backwardThrottle;

    public SimpleMotorBase(SpeedController motor, double forwardThrottle, double backwardThrottle) {
        motorController = motor;
        this.forwardThrottle = forwardThrottle;
        this.backwardThrottle = backwardThrottle;
        configureMotors();
    }

    public abstract void configureMotors();

    public void forward() {
        motorController.set(forwardThrottle);
    }

    public void backward() {
        motorController.set(-backwardThrottle);
    }

    public void stop() {
        motorController.set(0);
    }
}