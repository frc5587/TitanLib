package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class IntakeBase extends SubsystemBase {
    protected IntakeConstants constants;

    protected SpeedController leader;
    protected SpeedController[] followers;
    protected SpeedControllerGroup motorGroup;

    public abstract void configureMotors();

    public static class IntakeConstants {
        public int leaderID;
        public int followerIDs;
        public boolean inverted;
        public int stallLimit;
        public int freeLimit;
        public double throttle;

        public IntakeConstants(int leaderID, int followerIDs, boolean inverted, int stallLimit, int freeLimit, double throttle) {
            this.leaderID = leaderID;
            this.followerIDs = followerIDs;
            this.inverted = inverted;
            this.stallLimit = stallLimit;
            this.freeLimit = freeLimit;
            this.throttle = throttle;
        }
    }

    public void Intake(IntakeConstants constants, SpeedController[] motors) {
        followers = new SpeedControllerGroup[motors.length - 1];
        // put each motor in an array correspondoing to its type (leader or follower)
        for (int i = 0; i < motors.length; i++) {
            // if its first, its a leader
            if (i == 0) {
                // make a new motor with the ID and put it in the array
                this.leader = motors[i];
            } else {
                followers[i-1] = motors[i];
            }
        }

    // if there are no followers, don't put the array in the SpeedControllerGroup
    if (followers.length == 0) {
        motorGroup = new SpeedControllerGroup(leader);
    } else {
        motorGroup = new SpeedControllerGroup(leader, followers);
    }

    configureMotors();
    }
};