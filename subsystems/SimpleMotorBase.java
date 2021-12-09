package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SimpleMotorBase extends SubsystemBase {
    protected SpeedControllerGroup motorGroup;
    protected double throttle;

    public SimpleMotorBase(SpeedController[] motors, double throttle) {
        motorGroup = new SpeedControllerGroup(motors);
        this.throttle = throttle;
    configureMotors();
    }

    public abstract void configureMotors();

    public void forward() {
        motorGroup.set(throttle);
    };

    public void backward() {
        motorGroup.set(-throttle);
    };

    public void stop() {
        motorGroup.set(0);
    };
};
