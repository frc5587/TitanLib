package org.frc5587.lib.auto;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import org.frc5587.lib.subsystems.DifferentialDriveBase;

public class RamseteCommandWrapper extends Command {
    private final DifferentialDriveBase drivetrain; 
    private final Trajectory trajectory;
    private final RamseteConstants constants;

    private Command pathFollowCommand;
    private RamseteCommand ramsete;

    private boolean willZeroOdometry = false;
    private boolean willSetOdometryToFirstPose = false;

    private boolean debuggingMode = false;

    public static class RamseteConstants {
        public final SimpleMotorFeedforward ff;
        public final PIDController pid;
        public final double maxVelocity; // meters / s
        public final double maxAcceleration; // meters / s^2
        public final double maxRotationalAcceleration; // meters / s^2
        public final DifferentialDriveKinematics drivetrainKinematics;

        public RamseteConstants(double kS, double kV, double kA, double kP, double maxVelocity, double maxAcceleration,
                DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = new SimpleMotorFeedforward(kS, kV, kA);
            this.pid = new PIDController(kP, 0, 0);
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = 1;
            this.drivetrainKinematics = drivetrainKinematics;
        }

        public RamseteConstants(SimpleMotorFeedforward ff, PIDController pid, double maxVelocity, double maxAcceleration,
                DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = ff;
            this.pid = pid;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = 1;
            this.drivetrainKinematics = drivetrainKinematics;
        }

        public RamseteConstants(double kS, double kV, double kA, double kP, double maxVelocity, double maxAcceleration,
                double maxRotationalAcceleration, DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = new SimpleMotorFeedforward(kS, kV, kA);
            this.pid = new PIDController(kP, 0, 0);
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = maxRotationalAcceleration;
            this.drivetrainKinematics = drivetrainKinematics;
        }

        public RamseteConstants(SimpleMotorFeedforward ff, PIDController pid, double maxVelocity, double maxAcceleration,
                double maxRotationalAcceleration, DifferentialDriveKinematics drivetrainKinematics) {
            this.ff = ff;
            this.pid = pid;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.maxRotationalAcceleration = maxRotationalAcceleration;
            this.drivetrainKinematics = drivetrainKinematics;
        }
    }

    /**
     * Creates a new RamseteCommandWrapper from an {@link AutoPath}.
     */
    public RamseteCommandWrapper(DifferentialDriveBase drivetrain, AutoPath path, RamseteConstants constants) {
        this(drivetrain, path.trajectory, constants);
    }

    /**
     * Creates a new RamseteCommandWrapper from a {@link ConstrainedTrajectory}.
     */
    public RamseteCommandWrapper(DifferentialDriveBase drivetrain, ConstrainedTrajectory trajectory, RamseteConstants constants) {
        this(drivetrain, (Trajectory) trajectory, constants);
    }

    /**
     * Create a {@link RamseteCommand} from a trajectory
     * 
     * @param drivetrain drivetrain instance
     * @param trajectory trajectory of path
     * @param constants constants object
     */
    public RamseteCommandWrapper(DifferentialDriveBase drivetrain, Trajectory trajectory, RamseteConstants constants) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        this.constants = constants;

        makeRamsete();
    }

    /**
     * Generates a trajectory based on a start and end positions, and a list of waypoints. 
     * This should really only be used for testing and debugging, for more complex paths, PLEASE use Pathweaver.
     * Note: you cannot specify angle for waypoint
     * 
     * @param drivetrain drivetrain instance
     * @param start start position
     * @param path list of waypoints
     * @param end end position
     * @param constants constants object
     */
    public RamseteCommandWrapper(DifferentialDriveBase drivetrain, Pose2d start, List<Translation2d> path, Pose2d end,
            RamseteConstants constants) {
        this(drivetrain,
                TrajectoryGenerator.generateTrajectory(start, path, end,
                        new TrajectoryConfig(constants.maxVelocity, constants.maxAcceleration)
                                .setKinematics(constants.drivetrainKinematics)
                                .addConstraint(new DifferentialDriveVoltageConstraint(constants.ff,
                                        constants.drivetrainKinematics, 10))
                                .addConstraint(new CentripetalAccelerationConstraint(
                                        constants.maxRotationalAcceleration))),
                constants);
    }

    /**
     * Makes a new constrained trajectory from an existing trajectory.
     * 
     * @param drivetrain drietrain instance
     * @param originalTrajectory the trajectory to constrain
     * @param constants constants object
     * 
     * @return a new trajectory with the following constraints applied:
     * <ul>
     * <li>Maximum velocity</li>
     * <li>Maximum acceleration</li>
     * <li>Maximum voltage (hard set to 10 volts)</li>
     * <li>Maximum centripetal acceleration</li>
     * </ul>
     */
    public Trajectory makeConstrainedTrajectory(DifferentialDriveBase drivetrain, Trajectory originalTrajectory, RamseteConstants constants) {
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
                new TrajectoryConfig(constants.maxVelocity, constants.maxAcceleration)
                        .setKinematics(constants.drivetrainKinematics)
                        .addConstraint(new DifferentialDriveVoltageConstraint(constants.ff,
                                constants.drivetrainKinematics, 10))
                        .addConstraint(new CentripetalAccelerationConstraint(
                                constants.maxRotationalAcceleration)));

        return constrainedTrajectory;
    }

    /**
     * Creates the actual {@link RamseteCommand}. If this isn't called in the constructor, it means your robot will be sitting around for ~5ish second in the beginning of auto while it calculates the path.
     */
    private void makeRamsete() {
        NetworkTableEntry leftReference = SmartDashboard.getEntry("left_reference");
        NetworkTableEntry leftMeasurement = SmartDashboard.getEntry("left_measurement");
        NetworkTableEntry rightReference = SmartDashboard.getEntry("right_reference");
        NetworkTableEntry rightMeasurement = SmartDashboard.getEntry("right_measurement");

        RamseteController ramseteController = new RamseteController();

        PIDController left = constants.pid;
        PIDController right = constants.pid;

        if (debuggingMode) {
            /* 
            This "disables" the fancy control from the ramsete controller, allow you to
            verify the feedforward and PID gains. It is the same as doing what is
            suggested here:
            https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/troubleshooting.html#verify-feedforward 
            */
           
            // * Comment this line out to keep the controller enabled
            ramseteController.setEnabled(false);

            // * To test feedforward gains, uncomment these following lines (and make sure debugging mode is on
            
            // right.close();
            // left.close();
            // left = new PIDController(0, 0, 0);
            // right = new PIDController(0, 0, 0);
        }

        ramsete = new RamseteCommand(trajectory, drivetrain::getPose, ramseteController,
                constants.ff,
                constants.drivetrainKinematics, drivetrain::getWheelSpeeds,
                left,
                right,
                (leftVolts, rightVolts) -> {
                    drivetrain.tankDriveVolts(leftVolts, rightVolts);

                    if (debuggingMode) {
                        leftMeasurement.setNumber(drivetrain.getWheelSpeeds().leftMetersPerSecond);
                        leftReference.setNumber(left.getSetpoint());

                        rightMeasurement.setNumber(drivetrain.getWheelSpeeds().rightMetersPerSecond);
                        rightReference.setNumber(right.getSetpoint());
                    }
                },
                drivetrain);
    }

    /**
     * Sets the odometry to the origin right before running the path.
     * 
     * @return the command
     */
    public RamseteCommandWrapper zeroOdometryOnStart() {
        willZeroOdometry = true;
        return this;
    }

    /**
     * Resets the odometry to the first position of the path, right before running
     * the path
     * 
     * @return the command
     */
    public RamseteCommandWrapper setOdometryToFirstPoseOnStart() {
        willSetOdometryToFirstPose = true;
        return this;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
        if (willZeroOdometry) {
            drivetrain.zeroHeading();
            drivetrain.zeroOdometry();
        }

        if (willSetOdometryToFirstPose) {
            drivetrain.zeroHeading();
            drivetrain.setOdometry(trajectory.getInitialPose());
        }

        pathFollowCommand = ramsete;

        pathFollowCommand.schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain and path following command just in case
        if (pathFollowCommand != null) {
            pathFollowCommand.cancel();
        }
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return pathFollowCommand.isFinished();
    }

    /**
     * Turns on debugging mode, this will add the actual and reference values of the
     * velocities to {@link SmartDashboard} and it will "disable" the ramsete
     * controller, allowing you to verified the various gains
     * 
     * @param debuggingMode true for on
     * @return the controller so you can chain this
     */
    public RamseteCommandWrapper setDebuggingMode(boolean debuggingMode) {
        this.debuggingMode = debuggingMode;
        return this;
    }
}