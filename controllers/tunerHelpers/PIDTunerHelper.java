package org.frc5587.lib.controllers.tunerHelpers;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTunerHelper extends PIDController implements ControllerTunerHelper {
    private String pName;
    private String iName;
    private String dName;
    private String setpointName;
    private String measurementName;

    public PIDTunerHelper(String name, double kP, double kI, double kD) {
        super(kP, kI, kD);
        this.pName = name + " p";
        this.iName = name + " i";
        this.dName = name + " d";
        this.setpointName = name + " setpoint";
        this.measurementName = name + " measurement";
    }

    public void updateValues() {
        if (SmartDashboard.containsKey(pName)) {
            setP(SmartDashboard.getNumber(pName, Double.NaN));
        } else {
            SmartDashboard.putNumber(pName, getP());
        }

        if (SmartDashboard.containsKey(iName)) {
            setI(SmartDashboard.getNumber(iName, Double.NaN));
        } else {
            SmartDashboard.putNumber(iName, getI());
        }

        if (SmartDashboard.containsKey(dName)) {
            setD(SmartDashboard.getNumber(dName, Double.NaN));
        } else {
            SmartDashboard.putNumber(dName, getD());
        }

        if (SmartDashboard.containsKey(setpointName)) {
            setSetpoint(SmartDashboard.getNumber(setpointName, Double.NaN));
        } else {
            SmartDashboard.putNumber(setpointName, getSetpoint());
        }
    }

    @Override
    public double calculate(double measurement) {
        updateValues();

        SmartDashboard.putNumber(measurementName, measurement);

        return super.calculate(measurement);
    }
}
