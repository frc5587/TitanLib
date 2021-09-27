package org.frc5587.lib.controllers.tunerHelpers;

public interface ControllerTunerHelper {
    /**
     * Gets (or puts) the value from SmartDashboard and updates them internally
     */
    public void updateValues();

    /**
     * Calculates the controller gain based on `measurement` and returns it
     * 
     * @param measurement measurement of whatever controller
     * @return controller gain
     */
    public double calculate(double measurement);
}