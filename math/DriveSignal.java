package org.frc5587.lib.math;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 * 
 * This Code contains modified code from 254's 2019 repository, accessible at
 * https://github.com/Team254/FRC-2019-Public/blob/576b25d15cb51aebbc73d0b95b49d762bb96ebe1/src/main/java/com/team254/lib/util/DriveSignal.java
 */
public class DriveSignal {
    private final double mLeftMotor;
    private final double mRightMotor;
    private final boolean mBrakeMode;

    public DriveSignal(double left, double right) {
        this(left, right, false);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mBrakeMode = brakeMode;
    }

    public static DriveSignal fromControls(double throttle, double turn) {
        return new DriveSignal(throttle - turn, throttle + turn);
    }

    public static final DriveSignal NEUTRAL = new DriveSignal(0, 0);
    public static final DriveSignal BRAKE = new DriveSignal(0, 0, true);

    public double getLeft() {
        return mLeftMotor;
    }

    public double getRight() {
        return mRightMotor;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}