package org.frc5587.lib.pid;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTunerHelper extends PIDController {
    private String name;
    private String pName;
    private String iName;
    private String dName;
    private String setpointName;

    public PIDTunerHelper(String name, double kp, double ki, double kd) {
        super(kp, ki, kd);
        this.name = name;
        this.pName = name + " p";
        this.iName = name + " i";
        this.dName = name + " d";
        this.setpointName = name + " goto";
    }

    public void update() {
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
            SmartDashboard.putNumber(pName, getD());
        }

        if (SmartDashboard.containsKey(setpointName)) {
            setSetpoint(SmartDashboard.getNumber(setpointName, Double.NaN));
        } else {
            SmartDashboard.putNumber(setpointName, getSetpoint());
        }
    }

    @Override
    public double calculate(double measurement) {
        update();

        return super.calculate(measurement);
    }
}
