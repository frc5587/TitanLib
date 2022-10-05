package org.frc5587.lib.subsystems;

import org.frc5587.lib.math.DifferentialDrivePoseEstimator;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;

/**
 * A base drivetrain that includes support for arcade drive, tank drive,
 * and autonomous odometry.
 */
public abstract class DrivetrainBase extends SubsystemBase {
    /** create leader and follower motors for the drivetrain */
    protected MotorController left;
    protected MotorController right;

    /** make the speed controller groups into one drivetrain object */
    protected DifferentialDrive differentialDrive;

    /** create a DriveConstants object to look up constants */
    protected DriveConstants constants;

    /** create variables needed for odometry. */
    protected AHRS ahrs = new AHRS();
    protected boolean invertGyro;
    protected DifferentialDriveOdometry odometry;
    protected DifferentialDrivePoseEstimator odometryEstimator;
    protected final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5); 
    protected final DifferentialDriveKinematics kinematics;

    protected PIDController titanDrivePID;

    /**
     * A constants object that provides everything needed by {@link DrivetrainBase}
     */
    public static class DriveConstants {
        public final double wheelDiameterMeters, gearing, cpr, distancePerTick, trackWidth;
        public final int historyLimit;
        public final boolean invertGyro;

        /**
         * A constants object that provides everything needed by {@link DrivetrainBase}
         * 
         * @param wheelDiameterMeters the wheel diameter in meters
         * @param historyLimit        the limit of inputs for LimitedPoseMap
         * @param invertGyro          inverts values given by the gyroscope
         * @param cpr                 the motor encoders' counts per revolution
         * @param gearing             the gearing from the motor output to the wheels
         * @param trackWidth          the distance between the left and right wheels in
         *                            meters
         *                            (measured from the center of the width of the
         *                            wheel)
         */
        public DriveConstants(double wheelDiameterMeters, int historyLimit, boolean invertGyro, double cpr,
                double gearing, double trackWidth) {
            this.wheelDiameterMeters = wheelDiameterMeters;
            this.historyLimit = historyLimit;
            this.invertGyro = invertGyro;
            this.cpr = cpr;
            this.gearing = gearing;
            this.trackWidth = trackWidth;
            this.distancePerTick = ((1.0 / cpr) / gearing) * (Math.PI * wheelDiameterMeters);
        }
    }

    /**
     * A drivetrain that uses:
     * 
     * @param left      a MotorController for the left side of the drivetrain
     *                  (can be passed as a MotorController Group)
     * @param right     a MotorController for the right side of the drivetrain
     *                  (can be passed as a MotorController Group)
     * @param constants a {@link DriveConstants} object containing all constants
     *                  used by the class
     */
    public DrivetrainBase(MotorController left, MotorController right, DriveConstants constants) {
        this.constants = constants;
        this.left = left;
        this.right = right;

        /**
         * set all variables declared at the top to those given in the constructor
         * (mostly constants)
         */
        Rotation2d currentAngle = getRotation2d();
        this.odometry = new DifferentialDriveOdometry(currentAngle);
        this.odometryEstimator = new DifferentialDrivePoseEstimator(getRotation2d(), getPose(), // ! these numbers are 100% not tuned
                new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), // State measurement standard
                                                                                         // deviations. X, Y, theta.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations.
                                                                             // Left encoder, right encoder, gyro.
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));// Global measurement standard deviations. X,
                                                                           // Y, and theta.
        // this.poseHistory = new LimitedPoseMap(constants.historyLimit);
        this.invertGyro = constants.invertGyro;
        this.kinematics = new DifferentialDriveKinematics(constants.trackWidth);

        differentialDrive = new DifferentialDrive(left, right);
        differentialDrive.setDeadband(0);// Deadbanding is done in joystick
        configureMotors();
    }

    /**
     * The implementing class must configure the motors
     * <p>
     * This should do things like invert the motors, set their neutral mode, etc.
     */
    public abstract void configureMotors();

    /**
     * drive with a given throttle and curve (arcade drive)
     */
    public void arcadeDrive(double throttle, double curve) {
        differentialDrive.arcadeDrive(throttle, curve, false);
    }

    /**
     * Returns the follow direction for automatic following. So depending on the closest side to the desired heading, it either returns `FORWARD` or `BACKWARD`.
     * 
     * @param desiredHeading the desired heading for the drivetrain
     * @return the closest direction to follow
     */
    public FollowDirection getAutoFollowDirection(Rotation2d desiredHeading) {
        Rotation2d heading = getRotation2d();

        if (Math.abs(heading.minus(desiredHeading).getRadians()) < Math.PI/2) {
            return FollowDirection.FORWARD;
        } else {
            return FollowDirection.BACKWARD;
        }
    }

    /**
     * A field oriented drive system for differential drives.
     * 
     * @param desiredHeading direction to travel in (theta of joystick)
     * @param throttle power to apply (r of joystick)
     * @param followDirection whether to follow on the front, back, or automatically determine direction
     * @param spinInPlace whether to just spin
     */
    public void titanDrive(Rotation2d desiredHeading, double throttle, FollowDirection followDirection, boolean spinInPlace) {
        Rotation2d heading = getRotation2d();
        final Rotation2d forwardThreshold = Rotation2d.fromDegrees(60); // angle at which robot stops turning on a point, and starts moving in the direction

        if (followDirection == FollowDirection.AUTO) {
            followDirection = getAutoFollowDirection(desiredHeading);
        }

        if (followDirection == FollowDirection.BACKWARD) {
            throttle *= -1;
            heading = heading.plus(Rotation2d.fromDegrees(180));
        }

        Rotation2d angleError = desiredHeading.minus(heading);
        double spinFactor = spinInPlace? 0 : 1; 
;

        double left = throttle * (spinFactor - (angleError.getRadians() / forwardThreshold.getRadians()));
        double right = throttle * (spinFactor + (angleError.getRadians() / forwardThreshold.getRadians()));
        

        if (followDirection == FollowDirection.BACKWARD) {
            tankDrive(-right, -left);
        } else {
            tankDrive(-left, -right);
        }
    }

    /**
     * A field oriented drive system for differential drives using a PID controller instead of fuzzy logic.
     * 
     * @param desiredHeading direction to travel in (theta of joystick)
     * @param throttle power to apply (r of joystick)
     * @param followDirection whether to follow on the front, back, or automatically determine direction
     * @param spinInPlace whether to just spin
     */
    public void titanDrivePID(Rotation2d desiredHeading, double throttle, FollowDirection followDirection, boolean spinInPlace) {
        Rotation2d heading = getRotation2d();

        if (followDirection == FollowDirection.AUTO) {
            followDirection = getAutoFollowDirection(desiredHeading);
        }

        if (followDirection == FollowDirection.BACKWARD) {
            throttle *= -1;
            desiredHeading.rotateBy(Rotation2d.fromDegrees(180));
        }

        double output = titanDrivePID.calculate(heading.getRadians(), desiredHeading.getRadians());
        double spinFactor = spinInPlace? 0 : 1; 

        double left = throttle * (spinFactor - output);
        double right = throttle * (spinFactor + output);
        
        tankDrive(left, right);
    }

    public void setTitanDrivePID(PIDController titanDrivePID) {
        this.titanDrivePID = titanDrivePID;
    }

    public void curvatureDrive(double throttle, double curve, boolean quickTurn) {
        differentialDrive.curvatureDrive(throttle, curve, quickTurn);
    }

    /**
     * drive with a given throttle for each side of the robot (tank drive)
     */
    public void tankDrive(double leftThrottle, double rightThrottle) {
        differentialDrive.tankDrive(
                leftThrottle,
                rightThrottle,
                false);
    }

    /**
     * a tank drive that sets the voltages of the motors instead of throttle
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        left.setVoltage(leftVolts);
        right.setVoltage(rightVolts);
        differentialDrive.feed();
    }

    /**
     * an inverted version of tankDriveVolts
     */
    public void tankDriveVoltsReverse(double leftVolts, double rightVolts) {
        this.tankDriveVolts(-leftVolts, -rightVolts);
    }

    /**
     * sets the speeds of the MotorControllers rather than using the
     * DifferentialDrive
     */
    public void setThrottle(double speed) {
        left.set(speed);
        right.set(speed);
        differentialDrive.feed();
    }

    /**
     * stops all motors
     */
    public void stop() {
        setThrottle(0);
    }

    /**
     * @return Raw encoder ticks from motors for position
     */
    protected abstract double getRightPositionTicks();

    /**
     * @return Raw encoder ticks from motors for position
     */
    protected abstract double getLeftPositionTicks();

    /**
     * @return Raw encoder ticks per second from motors for velocity
     */
    protected abstract double getRightVelocityTicksPerSecond();

    /**
     * @return Raw encoder ticks per second from motors for velocity
     */
    protected abstract double getLeftVelocityTicksPerSecond();

    /**
     * Converts any fundamental tick value into meters:
     * <p>
     * ticks -> meters,
     * ticks per second -> meters per second,
     * ticks per second^2 -> meters per second,
     * etc
     * </p>
     *
     * @param rawTicks raw value read from encoders
     * @return value in meters
     */
    protected double getDistance(double rawTicks) {
        return rawTicks * constants.distancePerTick;
    }

    /**
     * @return The position in meters, converted from encoder ticks
     */
    public double getRightPositionMeters() {
        return getDistance(getRightPositionTicks());
    }

    /**
     * @return The position in meters, converted from encoder ticks
     */
    public double getLeftPositionMeters() {
        return getDistance(getLeftPositionTicks());
    }

    /**
     * @return The position in velocity in meters per second, converted from encoder
     *         ticks
     */
    public double getRightVelocityMetersPerSecond() {
        return getDistance(getRightVelocityTicksPerSecond());
    }

    /**
     * @return The position in velocity in meters per second, converted from encoder
     *         ticks
     */
    public double getLeftVelocityMetersPerSecond() {
        return getDistance(getLeftVelocityTicksPerSecond());
    }

    /**
     * Sets odometry to {@link Pose2d} specified. Notes: the rotation remains as
     * read from the AHRS.
     * 
     * @param pose position to set to
     */
    public void setOdometry(Pose2d pose) {
        resetEncoders();
        zeroHeading(); // ! I'm not sure if this is necessary, we'll see
        ahrs.setAngleAdjustment(pose.getRotation().getDegrees());

        odometry.resetPosition(pose, getRotation2d());
        odometryEstimator.resetPosition(pose, getRotation2d());
    }

    /**
     * Sets odometry to (0, 0)
     */
    public void zeroOdometry() {
        setOdometry(new Pose2d());
    }

    /**
     * Recalibrates the AHRS.
     * <p>
     * !Warning! this takes up to 10 seconds to complete (it recalibrates some
     * stuff), use sparingly
     */
    public void resetAHRS() {
        ahrs.reset();
    }

    /**
     * Adds an offset to the yaw (Z axis rotation) to zero it
     */
    public void zeroHeading() {
        ahrs.zeroYaw();
    }

    /**
     * @return the AHRS' position
     */
    public Rotation2d getRotation2d() {
        return ahrs.getRotation2d();
    }

    /**
     * @return the AHRS' position as a heading in degrees
     */
    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public double getAbsoluteHeadingRadians() {
        return Units.degreesToRadians(ahrs.getAngle());
    }

    /**
     * @return the angular velocity in radians
     */
    public double getAngularVelocity() {
        // multiplies by update rate because the documentation lies, in its actually in
        // degrees per update period (which is around 1/60 of a second), so multiplying
        // by update rate fixes that
        return Units.degreesToRadians(ahrs.getRate() * ahrs.getActualUpdateRate());
    }

    /**
     * @return the velocity of both sides of the drivetrain.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(),
                getRightVelocityMetersPerSecond());
    }

    /**
     * @return the X and Y velocities as a {@link ChassisSpeeds} object
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    /**
     * @return the linear velocity (x velocity) of the drivetrain in m/s
     */
    public double getLinearVelocity() {
        return getChassisSpeeds().vxMetersPerSecond;
    }

    /**
     * @return The position of the robot on the field (meters)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * If vision data is being updated, this could end up being more accurate than {@link DrivetrainBase#getPose}, otherwise it will be the exact same thing.
     * 
     * @return the estimated position of the robot on the field
     */
    public Pose2d getEstimatedPose() {
        return odometryEstimator.getEstimatedPosition();
    }

    /**
     * Uses the positioning based on vision data to correct for inconsistencies in the odometry
     * 
     * @param visionRobotPoseMeters estimated pose of the robot based on vision
     * @param timestampSeconds timestamp the vision is from
     */
    public void updateOdometryEstimatorWithVision(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        odometryEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public Pose2d getPoseAtTime(double FPGATimestamp) {
        return poseHistory.getSample(FPGATimestamp);
    }

    /**
     * resets the positions of the encoders to 0
     */
    protected abstract void resetEncoders();

    @Override
    public void periodic() {
        // Update the pose
        odometry.update(getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());
        odometryEstimator.update(getRotation2d(), getWheelSpeeds(), getLeftPositionMeters(), getRightPositionMeters());


        // Log the heading
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
    }

    public enum FollowDirection  {
        FORWARD, BACKWARD, AUTO;

        public FollowDirection opposite() {
            switch (this) {
                case FORWARD:
                    return BACKWARD;
                case BACKWARD:
                    return FORWARD;
                default:
                    throw new RuntimeException("No opposite to AUTO FollowDirection");
            }
        }
    }
}
