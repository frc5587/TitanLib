package org.frc5587.lib.pathfinder;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.SetValueMotionProfile;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import org.frc5587.lib.TitanDrive;
import org.frc5587.lib.TitanDrive.DriveSignal;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AbstractDrive extends SubsystemBase {
    protected TalonSRX leftMaster, rightMaster;
    protected IMotorController leftSlave, rightSlave;
    protected GyroBase gyro;
    protected AHRS ahrs;

    private double maxVelocity = 2500; // Max velocity in STU
    private int timeoutMS = 10;
    public int stuPerRev, stuPerInch, minBufferCount;
    public double wheelDiameterMeters;

    private MotionProfileStatus[] statuses = { new MotionProfileStatus(), new MotionProfileStatus() };
    public Notifier profileNotifer = new Notifier(new ProcessProfileRunnable());

    public AbstractDrive(TalonSRX leftMaster, TalonSRX rightMaster, IMotorController leftSlave,
            IMotorController rightSlave, boolean flipRight) {
        // Set motors to correct objects and connect the masters and slaves
        this.leftMaster = leftMaster;
        this.leftSlave = leftSlave;
        this.rightMaster = rightMaster;
        this.rightSlave = rightSlave;
        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        // Left should not be reversed
        leftMaster.setInverted(false);
        leftSlave.setInverted(false);

        // Invert the right side of the drivetrain
        if (flipRight) {
            rightMaster.setInverted(true);
            rightSlave.setInverted(true);
        } else {
            // The Talon might have been used in reversed state in past, so just to be sure
            rightMaster.setInverted(false);
            rightSlave.setInverted(false);
        }

        // Set encoder rotation as opposite direction relative to motor rotation
        this.leftMaster.setSensorPhase(true);
        this.rightMaster.setSensorPhase(true);

        this.ahrs = null;
        this.gyro = null;

        configPID(0);
        configSettings();
        enableBrakeMode(true);
    }

    public void setGyro(GyroBase gyro) {
        this.gyro = gyro;
    }

    public void setAHRS(AHRS ahrs) {
        this.ahrs = ahrs;
    }

    public void setConstants(double maxVelocity, int timeoutMS, int stuPerRev, int stuPerInch,
            double wheelDiameterMeters, int minBufferCount) {
        this.maxVelocity = maxVelocity;
        this.timeoutMS = timeoutMS;
        this.stuPerRev = stuPerRev;
        this.stuPerInch = stuPerInch;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.minBufferCount = minBufferCount;
    }

    public abstract void configPID(int slot);

    public abstract void configSettings();

    public void enableBrakeMode(boolean enabled) {
        if (enabled) {
            leftMaster.setNeutralMode(NeutralMode.Brake);
            rightMaster.setNeutralMode(NeutralMode.Brake);
            leftSlave.setNeutralMode(NeutralMode.Brake);
            rightSlave.setNeutralMode(NeutralMode.Brake);
        } else {
            leftMaster.setNeutralMode(NeutralMode.Coast);
            rightMaster.setNeutralMode(NeutralMode.Coast);
            leftSlave.setNeutralMode(NeutralMode.Coast);
            rightSlave.setNeutralMode(NeutralMode.Coast);
        }
    }

    /* --- BASIC MANUAL CONTROL CODE --- */

    public void vbusCurve(double throttle, double curve, boolean isQuickTurn) {
        DriveSignal d = TitanDrive.curvatureDrive(throttle, curve, isQuickTurn);

        leftMaster.set(ControlMode.PercentOutput, d.left);
        rightMaster.set(ControlMode.PercentOutput, d.right);
    }

    public void vbusArcade(double throttle, double turn) {
        DriveSignal d = TitanDrive.arcadeDrive(throttle, turn);

        leftMaster.set(ControlMode.PercentOutput, d.left);
        rightMaster.set(ControlMode.PercentOutput, d.right);
    }

    public void vbusLR(double left, double right) {
        leftMaster.set(ControlMode.PercentOutput, left);
        rightMaster.set(ControlMode.PercentOutput, right);
    }

    public void velocityCurve(double throttle, double curve, boolean isQuickTurn) {
        DriveSignal d = TitanDrive.curvatureDrive(throttle, curve, isQuickTurn);

        leftMaster.set(ControlMode.Velocity, d.left * maxVelocity);
        rightMaster.set(ControlMode.Velocity, d.right * maxVelocity);
    }

    public void velocityArcade(double throttle, double turn) {
        DriveSignal d = TitanDrive.arcadeDrive(throttle, turn);

        leftMaster.set(ControlMode.Velocity, d.left * maxVelocity);
        rightMaster.set(ControlMode.Velocity, d.right * maxVelocity);
    }

    public void stop() {
        leftMaster.neutralOutput();
        rightMaster.neutralOutput();
    }

    /* --- MOTION PROFILE HANDLING CODE --- */

    public class ProcessProfileRunnable implements java.lang.Runnable {
        public void run() {
            leftMaster.processMotionProfileBuffer();
            rightMaster.processMotionProfileBuffer();
        }
    }

    public void resetMP() {
        leftMaster.clearMotionProfileHasUnderrun(timeoutMS);
        leftMaster.clearMotionProfileTrajectories();
        leftMaster.changeMotionControlFramePeriod(10);
        leftMaster.configMotionProfileTrajectoryPeriod(10, timeoutMS);
        leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeoutMS);
        leftMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);

        rightMaster.clearMotionProfileHasUnderrun(timeoutMS);
        rightMaster.clearMotionProfileTrajectories();
        rightMaster.changeMotionControlFramePeriod(10);
        rightMaster.configMotionProfileTrajectoryPeriod(10, timeoutMS);
        rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, timeoutMS);
        rightMaster.set(ControlMode.MotionProfile, SetValueMotionProfile.Disable.value);
    }

    public void updateStatus() {
        leftMaster.getMotionProfileStatus(statuses[0]);
        rightMaster.getMotionProfileStatus(statuses[1]);
    }

    public boolean isMPReady() {
        boolean leftReady = getStatuses()[0].btmBufferCnt > minBufferCount;
        boolean rightReady = getStatuses()[0].btmBufferCnt > minBufferCount;
        return leftReady && rightReady;
    }

    public boolean isMPDone() {
        boolean leftDone = getStatuses()[0].isLast;
        boolean rightDone = getStatuses()[0].isLast;
        return leftDone && rightDone;
    }

    public void queuePoints(TrajectoryPoint[][] trajectories) {
        for (TrajectoryPoint point : trajectories[0]) {
            leftMaster.pushMotionProfileTrajectory(point);
        }
        for (TrajectoryPoint point : trajectories[1]) {
            rightMaster.pushMotionProfileTrajectory(point);
        }
    }

    public void setProfileMode(SetValueMotionProfile mpMode) {
        leftMaster.set(ControlMode.MotionProfile, mpMode.value);
        rightMaster.set(ControlMode.MotionProfile, mpMode.value);
    }

    /* --- UTILITY METHODS --- */

    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0, 0, timeoutMS);
        rightMaster.setSelectedSensorPosition(0, 0, timeoutMS);
    }

    public void sendDebugInfo() {
        SmartDashboard.putNumber("Left Distance", getLeftPosition());
        SmartDashboard.putNumber("Right Distance", getRightPosition());
        SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right Velocity", getRightVelocity());
        SmartDashboard.putNumber("Heading", getHeading());
    }

    public void sendMPDebugInfo() {
        SmartDashboard.putNumber("Left Expected Pos", leftMaster.getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Right Expected Pos", rightMaster.getActiveTrajectoryPosition());
        SmartDashboard.putNumber("Left Expected Vel", leftMaster.getActiveTrajectoryVelocity());
        SmartDashboard.putNumber("Right Expected Vel", rightMaster.getActiveTrajectoryVelocity());
    }

    /* --- GETTER METHODS --- */

    public int getLeftPosition() {
        return leftMaster.getSelectedSensorPosition(0);
    }

    public int getRightPosition() {
        return rightMaster.getSelectedSensorPosition(0);
    }

    public int getLeftVelocity() {
        return leftMaster.getSelectedSensorVelocity(0);
    }

    public int getRightVelocity() {
        return rightMaster.getSelectedSensorVelocity(0);
    }

    public double getLeftVoltage() {
        return leftMaster.getMotorOutputVoltage();
    }

    public double getRightVoltage() {
        return rightMaster.getMotorOutputVoltage();
    }

    public double getHeading() {
        if (ahrs != null) {
            return ahrs.getAngle();
        } else if (gyro != null) {
            return gyro.getAngle();
        } else {
            System.out.println(
                    "Neither the AHRS nor a Gyro were set for the drivetrain before attempting to read heading");
            return Double.NaN;
        }
    }

    public double getHeading(Double wrapValue) {
        var heading = getHeading() % 360;
        return ((heading > 180.0) ? (heading - 360.0) : ((heading < -180.0) ? (heading + 360.0) : heading));
    }

    public MotionProfileStatus[] getStatuses() {
        return statuses;
    }
}