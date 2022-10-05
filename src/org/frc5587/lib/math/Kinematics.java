package org.frc5587.lib.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a differential drive (with
 * a corrective factor to account for skidding).
 * 
 * This Code contains modified code from 254's 2019 repository, accessible at
 * https://github.com/Team254/FRC-2019-Public/blob/576b25d15cb51aebbc73d0b95b49d762bb96ebe1/src/main/java/com/team254/frc2019/Kinematics.java
 */
public class Kinematics {
    private static final double kEpsilon = 1E-9;

    /**
     * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but useful for predicting
     * motion)
     */
    public static Twist2d forwardKinematicsImpliedRotation(double left_wheel_delta, double right_wheel_delta, double trackWidth) {
        double delta_rotation = (right_wheel_delta - left_wheel_delta) / (trackWidth);
        return forwardKinematics(left_wheel_delta, right_wheel_delta, delta_rotation);
    }

    public static Twist2d forwardKinematics(double left_wheel_delta, double right_wheel_delta, double delta_rotation_rads) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        return new Twist2d(dx, 0.0, delta_rotation_rads);
    }

    public static Twist2d forwardKinematics(Rotation2d prev_heading, double left_wheel_delta, double right_wheel_delta,
                                            Rotation2d current_heading) {
        final double dx = (left_wheel_delta + right_wheel_delta) / 2.0;
        final double dy = 0.0;
        return new Twist2d(dx, dy, prev_heading.unaryMinus().rotateBy(current_heading).getRadians());
    }

    /**
     * For convenience, integrate forward kinematics with a Twist2d and previous rotation.
     */
    public static Pose2d integrateForwardKinematics(Pose2d current_pose,
                                                    Twist2d forward_kinematics) {
        return current_pose.exp(forward_kinematics);
    }

    /**
     * Uses inverse kinematics to convert a Twist2d into left and right wheel velocities
     */
    public static DriveSignal inverseKinematics(Twist2d velocity, double trackWidth) {
        if (Math.abs(velocity.dtheta) < kEpsilon) {
            return new DriveSignal(velocity.dx, velocity.dx);
        }
        double delta_v = trackWidth * velocity.dtheta / 2;
        return new DriveSignal(velocity.dx - delta_v, velocity.dx + delta_v);
    }
}