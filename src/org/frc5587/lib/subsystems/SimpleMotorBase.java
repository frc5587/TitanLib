package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SimpleMotorBase extends SubsystemBase {
    protected MotorController motors;
    protected double forwardThrottle;
    protected double backwardThrottle;

    public SimpleMotorBase(MotorController motors, double forwardThrottle, double backwardThrottle) {
        // Create MotorControllerGroup with the motors passed in as a parameter
        this.motors = motors;
        this.forwardThrottle = forwardThrottle;
        this.backwardThrottle = backwardThrottle;
        configureMotors();
    }

    public abstract void configureMotors();

    public void forward() {
        motors.set(forwardThrottle);
    }

    public void backward() {
        motors.set(-backwardThrottle);
    }

    public void stop() {
        motors.set(0);
    }
}