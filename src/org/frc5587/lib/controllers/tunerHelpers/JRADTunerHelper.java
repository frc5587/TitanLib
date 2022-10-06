package org.frc5587.lib.controllers.tunerHelpers;

import org.frc5587.lib.controllers.JRADShooterController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JRADTunerHelper extends JRADShooterController implements ControllerTunerHelper {
    private String fName;
    private String jName;
    private String loadRatioName;
    private String setpointName;
    private String measurementName;

    public JRADTunerHelper(String name, double kF, double kJ, double kLoadRatio) {
        super(kF, kJ, kLoadRatio);
        this.fName = name + " f";
        this.jName = name + " j";
        this.loadRatioName = name + " loadratio";
        this.setpointName = name + " setpoint";
        this.measurementName = name + " measurement";
    }

    public void updateValues() {
        if (SmartDashboard.containsKey(fName)) {
            setF(SmartDashboard.getNumber(fName, Double.NaN));
        } else {
            SmartDashboard.putNumber(fName, getF());
        }

        if (SmartDashboard.containsKey(jName)) {
            setJ(SmartDashboard.getNumber(jName, Double.NaN));
        } else {
            SmartDashboard.putNumber(jName, getJ());
        }

        if (SmartDashboard.containsKey(loadRatioName)) {
            setLoadRatio(SmartDashboard.getNumber(loadRatioName, Double.NaN));
        } else {
            SmartDashboard.putNumber(loadRatioName, getLoadRatio());
        }

        if (SmartDashboard.containsKey(setpointName)) {
            setSetpoint(SmartDashboard.getNumber(setpointName, Double.NaN));
        } else {
            SmartDashboard.putNumber(setpointName, getSetpoint());
        }
    }

    @Override
    public double calculate(double currentVelocityRPS) {
        updateValues();

        SmartDashboard.putNumber(measurementName, currentVelocityRPS);

        return super.calculate(currentVelocityRPS);
    }
}
