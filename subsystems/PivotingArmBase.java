package org.frc5587.lib.subsystems;

import org.frc5587.lib.pid.PID;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SpeedController;

public abstract class PivotingArmBase extends SubsystemBase {
    protected ArmsConstants constants;
    protected WPI_TalonFX leader;
    protected SpeedControllerGroup motorGroup;
    protected WPI_TalonFX[] followers;

    public static class ArmsConstants {
        public double armSpeedMultiplier, armArcDiameter;
        public int limitSwitchPort, pidSlot;
        public PID pid;
        public ArmFeedforward ff;

        public ArmsConstants(double armSpeedMultiplier, double armArcDiameter, int limitSwitchPort, int pidSlot, PID pid, ArmFeedforward ff) {
            this.armSpeedMultiplier = armSpeedMultiplier;
            this.armArcDiameter = armArcDiameter;
            this.limitSwitchPort = limitSwitchPort;
            this.pidSlot = pidSlot;
            this.pid = pid;
            this.ff = ff;
        }
    }

    public PivotingArmBase(ArmsConstants constants, int[] motorIDs) {
        super();
        this.constants = constants;
        followers = new WPI_TalonFX[motorIDs.length-1];
        // put each motor in an array corresponding to its type (leader or follower)
        for(int i = 0; i < motorIDs.length; i++) {
            // if it's the first one in the array, it's a leader
            if(i == 0) {
                // make a new motor with the given ID and put it in the array
                this.leader = new WPI_TalonFX(motorIDs[i]);
            }
            // otherwise it's a follower
            else {
                followers[i-1] = new WPI_TalonFX(motorIDs[i]);
            }
        }

        // if there are no followers, don't put the array in the speedcontroller group
        if(followers.length == 0) {
            //motorGroup = new SpeedControllerGroup(leader);
        }
        else {
            motorGroup = new SpeedControllerGroup((SpeedController) leader, (SpeedController[]) followers);
        }

        leader.config_kP(constants.pidSlot, constants.pid.kP);
        leader.config_kI(constants.pidSlot, constants.pid.kI);
        leader.config_kD(constants.pidSlot, constants.pid.kD);
        leader.config_kF(constants.pidSlot, calcFeedForward());
        configureMotors();
    }

    public abstract void configureMotors();

    // move the arm based on a given throttle 
    public void moveArmThrottle(double throttle) {
        motorGroup.set(throttle * constants.armSpeedMultiplier);
    }

    // move the arm based on a constant multiplier (for operation with buttons)
    public void moveArmFixedSpeed() {
        motorGroup.set(constants.armSpeedMultiplier);
    }

    // reverse the above function (for operation with buttons)
    public void moveArmFixedReversed() {
        motorGroup.set(-constants.armSpeedMultiplier);
    }

    // moves the arm based on voltage instead of speed
    public void moveArmVolts(double voltage) {
        motorGroup.setVoltage(voltage);
    }

    public void moveArmVoltsReversed(double voltage) {
        motorGroup.setVoltage(-voltage);
    }

    public double calcFeedForward() {
        double ff = constants.ff.calculate(Math.toRadians(SmartDashboard.getNumber("Goto Position", 0)), 0) / 12;
        return ff;
    }

    public void startPID() {
        SmartDashboard.putNumber("Goto Position", 14);
    }

    public void refreshPID() {
        SmartDashboard.putNumber("Angle", getPositionDegrees());
        SmartDashboard.putNumber("Encoder Val", getPositionRotation());
        SmartDashboard.putNumber("FF", calcFeedForward());
        SmartDashboard.putNumber("Vel", getVelocityDegreesPerSecond());
    }
    
    // gets the encoder's position
    public double getPositionRotation() {
        return leader.getSelectedSensorPosition();
    }

    public double getPositionDegrees() {
        return getPositionRotation() * 180;
    }

    // gets the arm's velocity in rotations per minute
    public double getVelocityRPM() {
        return leader.getSelectedSensorVelocity(); // TODO: Removed / 2. Check if correct.
    }

    public double getVelocityDegreesPerSecond() {
        return getVelocityRPM() * 180;
    }

    // converts rotations per minute (RPM) to meters per second (MPS)
    private double rpmToMPS(double rotationsPerMinute) {
        double radiansPerSecond = Units.rotationsPerMinuteToRadiansPerSecond(rotationsPerMinute);
        double linearMetersPerSecond = radiansPerSecond * (constants.armArcDiameter / 2);
        return linearMetersPerSecond;
    }

    // gets the arm's velocity in meters per second
    public double getVelocityMPS() {
        return rpmToMPS(getVelocityRPM());
    }

    public void resetEncoders() {
        leader.setSelectedSensorPosition(0);
    }

    // stops the speedcontroller group
    public void stop() {
        motorGroup.set(0);
    }

    // stops every motor without going through the speedcontroller group
    public void stopMotors() {
        leader.set(0);
        for(WPI_TalonFX follower : followers) {
            follower.set(0);
        }
    }
}
