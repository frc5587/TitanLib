package org.frc5587.lib.auto;

import java.util.List;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SuperSimpleDrivetrain;

import org.frc5587.lib.subsystems.DrivetrainBase;

public class RamseteCommandWrapper extends CommandBase {
    private final SuperSimpleDrivetrain drivetrain; // TODO remember to change this back
    private final Trajectory trajectory;
    private final RamseteConstants constants;

    private Command pathFollowCommand;
    private RamseteCommand ramsete;

    private boolean willZeroOdometry = false;
    private boolean willResetOdometry = false;

    private boolean debuggingMode = false;

    public static class RamseteConstants {
        public final double kS; // volts
        public final double kV; // volts * seconds / meters
        public final double kA; // volts * seconds / meters^2
        public final double kP;
        public final double maxVelocity; // meters / s
        public final double maxAcceleration; // meters / s^2
        public final DifferentialDriveKinematics drivetrainKinematics;

        public RamseteConstants(double kS, double kV, double kA, double kP, double maxVelocity, double maxAcceleration,
                DifferentialDriveKinematics drivetrainKinematics) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.kP = kP;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.drivetrainKinematics = drivetrainKinematics;
        }
    }

    /**
     * Creates a new RamseteCommandWrapper.
     */
    public RamseteCommandWrapper(SuperSimpleDrivetrain drivetrain, AutoPath path, RamseteConstants constants) {
        this(drivetrain, path.trajectory, constants);
    }

    public RamseteCommandWrapper(SuperSimpleDrivetrain drivetrain, Trajectory trajectory, RamseteConstants constants) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        this.constants = constants;

        makeRamsete();
    }

    public RamseteCommandWrapper(SuperSimpleDrivetrain drivetrain, Pose2d start, List<Translation2d> path, Pose2d end,
            RamseteConstants constants) {
        this(drivetrain,
                TrajectoryGenerator.generateTrajectory(start, path, end,
                        new TrajectoryConfig(constants.maxVelocity, constants.maxAcceleration)
                                .setKinematics(constants.drivetrainKinematics)
                                .addConstraint(new DifferentialDriveVoltageConstraint(
                                        new SimpleMotorFeedforward(constants.kS, constants.kV,
                                                constants.kA),
                                        constants.drivetrainKinematics, 10))),
                constants);
    }

    private void makeRamsete() {
        NetworkTableEntry leftReference = SmartDashboard.getEntry("left_reference");
        NetworkTableEntry leftMeasurement = SmartDashboard.getEntry("left_measurement");
        NetworkTableEntry rightReference = SmartDashboard.getEntry("right_reference");
        NetworkTableEntry rightMeasurement = SmartDashboard.getEntry("right_measurement");

        RamseteController ramseteController = new RamseteController();

        PIDController left = new PIDController(AutoConstants.KP, 0, 0);
        PIDController right = new PIDController(AutoConstants.KP, 0, 0);

        if (debuggingMode) {
            // This "disables" the fancy control from the ramsete controller, allow you to
            // verify the feedforward and PID gains. It is the same as doing what is
            // suggested here:
            // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/troubleshooting.html#verify-feedforward
            // * you can also comment this line out if you don't want to disable the controller
            ramseteController.setEnabled(false);

            // * To test feedforward gains, uncomment these following lines (and make sure
            // debugging mode is on)
            // right.close();
            // left.close();
            // left = new PIDController(0, 0, 0);
            // right = new PIDController(0, 0, 0);
        }

        ramsete = new RamseteCommand(trajectory, drivetrain::getPose, ramseteController,
                new SimpleMotorFeedforward(constants.kS, constants.kV,
                        constants.kA),
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
    public RamseteCommandWrapper resetOdometryOnStart() {
        willResetOdometry = true;
        return this;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (willZeroOdometry) {
            drivetrain.zeroOdometry();
        }

        if (willResetOdometry) {
            drivetrain.resetOdometry(trajectory.getInitialPose());
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