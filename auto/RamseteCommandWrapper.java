package org.frc5587.lib.auto;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import org.frc5587.lib.subsystems.DrivetrainBase;

public class RamseteCommandWrapper extends CommandBase {
    private final DrivetrainBase drivetrain;
    private final Trajectory trajectory;
    private final RamseteConstants constants;

    private Command pathFollowCommand;
    private RamseteCommand ramsete;

    private boolean willZeroOdometry = false;
    private boolean willResetOdometry = false;

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
    public RamseteCommandWrapper(DrivetrainBase drivetrain, AutoPath path, RamseteConstants constants) {
        this(drivetrain, path.trajectory, constants);
    }

    public RamseteCommandWrapper(DrivetrainBase drivetrain, Trajectory trajectory, RamseteConstants constants) {
        addRequirements(drivetrain);

        this.drivetrain = drivetrain;
        this.trajectory = trajectory;
        this.constants = constants;

        makeRamsete();
    }

    public RamseteCommandWrapper(DrivetrainBase drivetrain, Pose2d start, List<Translation2d> path, Pose2d end,
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
        ramsete = new RamseteCommand(trajectory, drivetrain::getPose, new RamseteController(),
                new SimpleMotorFeedforward(constants.kS, constants.kV,
                        constants.kA),
                constants.drivetrainKinematics, drivetrain::getWheelSpeeds,
                new PIDController(constants.kP, 0, 0),
                new PIDController(constants.kP, 0, 0),
                drivetrain::tankDriveVolts, drivetrain);
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
     * Resets the odometry to the first position of the path, right before running the path
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
        
        // Start the pathFollowCommand
        
        if (willZeroOdometry) {
            drivetrain.resetOdometry();
        }

        if (willResetOdometry) {
            drivetrain.resetOdometry(trajectory.getInitialPose());
            System.out.println(trajectory.getInitialPose());
        }

        // TODO: try transformting the whole path if nothing else works

        pathFollowCommand = ramsete;

        pathFollowCommand.schedule();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        super.execute();
        // ramsete.
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
}