package org.frc5587.lib.auto;

import java.util.List;

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
        // var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
        var leftReference = SmartDashboard.getEntry("left_reference");
        var leftMeasurement = SmartDashboard.getEntry("left_measurement");
        var rightReference = SmartDashboard.getEntry("right_reference");
        var rightMeasurement = SmartDashboard.getEntry("right_measurement");
        RamseteController disabledRamsete = new RamseteController();/* new RamseteController() {
            @Override
            public ChassisSpeeds calculate(Pose2d currentPose, Pose2d poseRef, double linearVelocityRefMeters,
                    double angularVelocityRefRadiansPerSecond) {
                return new ChassisSpeeds(linearVelocityRefMeters, 0.0, angularVelocityRefRadiansPerSecond);
            }
        }; */

        var left = new PIDController(constants.kP, 0, 0);
        var right =new PIDController(constants.kP, 0, 0);

        ramsete = new RamseteCommand(trajectory, drivetrain::getPose, disabledRamsete,
                new SimpleMotorFeedforward(constants.kS, constants.kV,
                        constants.kA),
                constants.drivetrainKinematics, drivetrain::getWheelSpeeds,
                left,
                right,
                // drivetrain::tankDriveVolts
                (leftVolts, rightVolts) -> {
                    drivetrain.tankDriveVolts(leftVolts, rightVolts);
            
                    leftMeasurement.setNumber(drivetrain.getWheelSpeeds().leftMetersPerSecond);
                    leftReference.setNumber(left.getSetpoint());
            
                    rightMeasurement.setNumber(drivetrain.getWheelSpeeds().rightMetersPerSecond);
                    rightReference.setNumber(right.getSetpoint());
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
        System.out.println("starting...");
        // Start the pathFollowCommand
        
        if (willZeroOdometry) {
            drivetrain.zeroOdometry();
        }

        if (willResetOdometry) {
            drivetrain.resetOdometry(trajectory.getInitialPose());
            // System.out.println(trajectory.getInitialPose());
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

    @Override
    public boolean isFinished() {
        return pathFollowCommand.isFinished();
    }
}