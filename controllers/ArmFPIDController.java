package org.frc5587.lib.controllers;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;

public class ArmFPIDController extends PIDController {
    protected double f, p, i, d;
    private ArmFeedforward armFF;
    private boolean fDisabled = false;

    // params fpid are Feedforward and PID gains, param sCosVA is an array of the ks, kcos, kv, and ka used by an ArmFeedForward
    // all values should come from robot characterization.
    public ArmFPIDController(double f, double p, double i, double d, double[] sCosVA) {
        super(p, i, d);
        this.f = f;
        this.p = p;
        this.i = i;
        this.d = d;
        armFF = new ArmFeedforward(sCosVA[0], sCosVA[1], sCosVA[2], sCosVA[3]);
    }

    // you can also pass an ArmFeedforward instead of an array of the values needed to create one
    public ArmFPIDController(double f, double p, double i, double d, ArmFeedforward armFF) {
        super(p, i, d);
        this.f = f;
        this.p = p;
        this.i = i;
        this.d = d;
        this.armFF = armFF;
    }

    // returns true if Feedforward is enabled
    public boolean isFEnabled() {
        return !fDisabled;
    }

    // disables Feedforward control/integration
    public void disableFeedForward() {
        fDisabled = true;
    }

    // enables Feedforward control/integration
    public void enableFeedForward() {
        fDisabled = false;
    }

    // returns the Feedforward gain
    public double getF() {
        return f;
    }

    // sets the Feedforward gain
    public void setF(double f) {
        this.f = f;
    }

    // sets the Feedfoward and PID gains
    public void setFPID(double f, double p, double i, double d) {
        this.setPID(p, i, d);
        this.f = f;
    }

    // calculates the Feedforward value based on position and velocity
    public double calculateF(double position, double velocity) {
        return armFF.calculate(position, velocity);
    }

    // returns the ArmFeedforward armFF of this type in case it is needed
    public ArmFeedforward getArmFeedforward() {
        return armFF;
    }

    public double calculate(double measurement, double position, double velocity) {
        // if Feedforward is disabled, only return the result of PIDController's calculate() method
        if(fDisabled) {
            System.out.println("Tried to calculate FPID with Feedforward disabled. Use PIDController.calculate() instead.");
            return super.calculate(measurement);
        }
        else {
            // returns the calculated PID from PIDController multiplied by the calculated Feedforward value.
            return super.calculate(measurement) * calculateF(position, velocity);
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("f", this::getF, this::setF);
        super.initSendable(builder);
    }
}
