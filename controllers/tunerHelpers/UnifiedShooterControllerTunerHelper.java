package org.frc5587.lib.controllers.tunerHelpers;

import org.frc5587.lib.controllers.UnifiedShooterController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class UnifiedShooterControllerTunerHelper extends UnifiedShooterController implements ControllerTunerHelper {

    private String fName;
    private String jName;
    private String uName;
    private String nName;
    private String pName;
    private String setpointName;
    private String distanceName;
    private String measurementName;
    private String outputName;

    public UnifiedShooterControllerTunerHelper(String name, double kF, double kJ, double kU, double kN, double kP) {
        super(kF, kJ, kU, kN, kP);
        this.fName = name + " f";
        this.jName = name + " j";
        this.uName = name + " u";
        this.nName = name + " n";
        this.pName = name + " p";
        this.setpointName = name + " setpoint";
        this.distanceName = name + " distance";
        this.outputName = name + " output";
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

        if (SmartDashboard.containsKey(uName)) {
            setU(SmartDashboard.getNumber(uName, Double.NaN));
        } else {
            SmartDashboard.putNumber(uName, getU());
        }

        if (SmartDashboard.containsKey(nName)) {
            setN(SmartDashboard.getNumber(nName, Double.NaN));
        } else {
            SmartDashboard.putNumber(nName, getN());
        }

        if (SmartDashboard.containsKey(pName)) {
            setP(SmartDashboard.getNumber(pName, Double.NaN));
        } else {
            SmartDashboard.putNumber(pName, getP());
        }

        if (SmartDashboard.containsKey(distanceName)) {
            setDistance(SmartDashboard.getNumber(distanceName, Double.NaN));
        } else {
            SmartDashboard.putNumber(distanceName, getDistance());
        }

        SmartDashboard.putNumber(setpointName, getSetpoint());
        SmartDashboard.putNumber(outputName, getLastOutput());
    }

    @Override
    public double calculate(double currentVelocityRPS) {
        updateValues();

        SmartDashboard.putNumber(measurementName, currentVelocityRPS);

        return super.calculate(currentVelocityRPS);
    }
    
}
