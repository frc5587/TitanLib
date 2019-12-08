package org.frc5587.lib.pathfinder;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GyroBase;

interface AbstractDrive {

    public void setGyro(GyroBase gyro);

    public void setAHRS(AHRS ahrs);

    public void setConstants(double maxVelocity, int timeoutMS, double wheelDiameterMeters, int minBufferCount);

    public void enableBrakeMode(boolean enabled);

    /* --- BASIC MANUAL CONTROL CODE --- */

    public void vbusCurve(double throttle, double curve, boolean isQuickTurn);

    public void vbusArcade(double throttle, double turn);

    public void vbusLR(double left, double right);

    public void velocityCurve(double throttle, double curve, boolean isQuickTurn);

    public void velocityArcade(double throttle, double turn);

    public void stop();

    /* --- MOTION PROFILE HANDLING CODE --- */

    /* --- UTILITY METHODS --- */

    public void resetEncoders();

    public void sendDebugInfo();

    public void sendMPDebugInfo();

    /* --- GETTER METHODS --- */

    public int getLeftPosition();

    public int getRightPosition();

    public int getLeftVelocity();

    public int getRightVelocity();

    public double getLeftVoltage();

    public double getRightVoltage();

    public double getHeading();

    public double getHeading(Double wrapValue);

}