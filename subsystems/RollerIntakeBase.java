package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class RollerIntakeBase extends SubsystemBase {
    protected IntakeConstants constants;

    // 1 leader variable is utilized, while an array for followers is used for ease of use
    protected SpeedController leader;
    protected SpeedController[] followers;
    protected SpeedControllerGroup motorGroup;

    // Create intake constants for implementation in methods
    public static class IntakeConstants {
        public boolean inverted;
        public int stallLimit;
        public int freeLimit;
        public double throttle;

        public IntakeConstants(int leaderID, int followerIDs, boolean inverted, int stallLimit, int freeLimit, double throttle) {
            this.inverted = inverted;
            this.stallLimit = stallLimit;
            this.freeLimit = freeLimit;
            this.throttle = throttle;
        }
    }

    public RollerIntakeBase(IntakeConstants constants, SpeedController[] motors) {
        // Add motors to a SpeedControllerGroup, minus 1 to remove the leader
        followers = new SpeedControllerGroup[motors.length - 1];
        this.constants = constants;
        // put each motor in an array corresponding to its type (leader or follower)
        for (int i = 0; i < motors.length; i++) {
            // if its first, its a leader
            if (i == 0) {
                // make a new motor with the ID and put it in the array
                this.leader = motors[i];
            // if its not, its a follower
            } else {
                // put non-leader motor into an array as followers
                followers[i-1] = motors[i];
            }
        }

        // if there are no followers, don't put the followers array in the SpeedControllerGroup
        if (followers.length == 0) {
            motorGroup = new SpeedControllerGroup(leader);
        } else {
            motorGroup = new SpeedControllerGroup(leader, followers);
        }
        
    configureMotors();
    }

    public abstract void configureMotors();

    public void forward() {
        motorGroup.set(constants.throttle);
    };

    public void backward() {
        motorGroup.set(-constants.throttle);
    };

    public void stop() {
        motorGroup.set(0);
    };
};
