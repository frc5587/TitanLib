package org.frc5587.lib.auto;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

public class ConstrainedTrajectory extends Trajectory {
    public final Trajectory trajectory;
    public final TrajectoryConstraints constraints;

    public static class TrajectoryConstraints {
        public final double maxVelocity; // meters / s
        public final double maxAcceleration; // meters / s^2
        public final double maxRotationalAcceleration; // meters / s^2
        public final SimpleMotorFeedforward ff;
        public final DifferentialDriveKinematics drivetrainKinematics;

        public TrajectoryConstraints(double kS, double kV, double kA, double maxVelocity, double maxAcceleration,
                DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = new SimpleMotorFeedforward(kS, kV, kA);
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = 1;
            this.drivetrainKinematics = drivetrainKinematics;
        }

        public TrajectoryConstraints(SimpleMotorFeedforward ff, double maxVelocity, double maxAcceleration,
                DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = ff;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = 1;
            this.drivetrainKinematics = drivetrainKinematics;
        }

        public TrajectoryConstraints(double kS, double kV, double kA, double kP, double maxVelocity,
                double maxAcceleration, double maxRotationalAcceleration,
                DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = new SimpleMotorFeedforward(kS, kV, kA);
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = maxRotationalAcceleration;
            this.drivetrainKinematics = drivetrainKinematics;
        }

        public TrajectoryConstraints(SimpleMotorFeedforward ff, double maxVelocity, double maxAcceleration,
                double maxRotationalAcceleration, DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = ff;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = maxRotationalAcceleration;
            this.drivetrainKinematics = drivetrainKinematics;
        }
    }

    public ConstrainedTrajectory(Trajectory originalTrajectory, TrajectoryConstraints constraints) {
        this.trajectory = constrain(originalTrajectory, constraints);
        this.constraints = constraints;

        super.getStates().clear();
        super.getStates().addAll(trajectory.getStates());
    }

    /**
     * Makes a new constrained trajectory from an existing trajectory.
     * 
     * @param drivetrain drietrain instance
     * @param originalTrajectory the trajectory to constrain
     * @param constraints constraints object
     * 
     * @return a new trajectory with the following constraints applied:
     * <ul>
     * <li>Maximum velocity</li>
     * <li>Maximum acceleration</li>
     * <li>Maximum voltage (hard set to 10 volts)</li>
     * <li>Maximum centripetal acceleration</li>
     * </ul>
     */
    public static Trajectory constrain(Trajectory originalTrajectory, TrajectoryConstraints constraints) {
        List<State> allStates = originalTrajectory.getStates();
        List<Pose2d> allPoses = new ArrayList<Pose2d>(); 

        /**
         * Make a new list of poses from the old trajectory states 
         */
        for(int i = 0; i <= allStates.size(); i++) {
            allPoses.add(allStates.get(i).poseMeters);
        }

        /**
         * Generate a new trajectory using TrajectoryGenerator with the 
         * list of all translations within the existing trajectory
         */
        Trajectory constrainedTrajectory = TrajectoryGenerator.generateTrajectory(
                allPoses,
                new TrajectoryConfig(constraints.maxVelocity, constraints.maxAcceleration)
                        .setKinematics(constraints.drivetrainKinematics)
                        .addConstraint(new DifferentialDriveVoltageConstraint(constraints.ff,
                                constraints.drivetrainKinematics, 10))
                        .addConstraint(new CentripetalAccelerationConstraint(
                                constraints.maxRotationalAcceleration)));

        return constrainedTrajectory;
    }

    public void constrain(TrajectoryConstraints constraints) {
        constrain(this.trajectory, constraints);
    }
}
