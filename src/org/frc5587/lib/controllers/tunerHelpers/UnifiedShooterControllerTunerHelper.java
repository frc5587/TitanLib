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
    private String spinName;

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
        this.spinName = name + " spin to";
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

        if (!SmartDashboard.containsKey(spinName)) {
            SmartDashboard.putNumber(spinName, 0);
        }

        SmartDashboard.putNumber(setpointName, getSetpoint());
        SmartDashboard.putNumber(outputName, getLastOutput());
    }

    /**
     * Allows someone to set the value on SmartDashboard in order to tune the UNP
     * regression model. If the SmartDashboard value of `spinName` is not 0, then it
     * will spin up to that value, otherwise it will go with whatever the current
     * UNP model says.
     */
    @Override
    public double calculateUNP(double distance) {
        if (SmartDashboard.getNumber(spinName, 0) != 0) {
            return SmartDashboard.getNumber(spinName, 0);
        } else {
            return super.calculateUNP(distance);
        }
    }

    @Override
    public double calculate(double currentVelocityRPS) {
        updateValues();

        SmartDashboard.putNumber(measurementName, currentVelocityRPS);

        return super.calculate(currentVelocityRPS);
    }

}
