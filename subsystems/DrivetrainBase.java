package org.frc5587.lib.subsystems;

import org.frc5587.lib.advanced.LimitedPoseMap;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public abstract class DrivetrainBase extends SubsystemBase {
    // create each motors side for the drivetrain
    protected SpeedController left;
    protected SpeedController right;

    // make the speed controller groups into one drivetrain object
    protected DifferentialDrive differentialDrive;

    // create a hashtable to look up constants
    protected DriveConstants constants;

    // create variables needed for odometry.
    protected AHRS ahrs = new AHRS();
    protected boolean invertGyro;
    protected DifferentialDriveOdometry odometry;
    private final LimitedPoseMap poseHistory;

    public static class DriveConstants {
        public final double wheelDiameterMeters, gearing, epr, distancePerTick;
        public final int historyLimit, flipLeft, flipRight;
        public final boolean invertGyro;

        public DriveConstants(double wheelDiameterMeters, int historyLimit, boolean invertGyro, double epr, double gearing, boolean flipLeft, boolean flipRight) {
            this.wheelDiameterMeters = wheelDiameterMeters;
            this.historyLimit = historyLimit;
            this.invertGyro = invertGyro;
            this.epr = epr;
            this.gearing = gearing;
            this.flipLeft = flipLeft? -1 : 1;
            this.flipRight = flipRight? -1 : 1;
            this.distancePerTick = ((1.0 / epr) / gearing) * (Math.PI * wheelDiameterMeters); 
        }
    }

    // set all of the variables from the subclass to this abstract class
    // MotorIDs: a arrays of integer CAN IDs used to make motors. index 0 should ALWAYS be the leader motor, and anything else is a follower.
    public DrivetrainBase(SpeedController left, SpeedController right, DriveConstants constants) {       
        this.constants = constants;
        this.left = left;
        this.right = right;

        /* VARIABLE DECLARATION */
        // set all variables declared at the top to those given in the constructor (mostly constants)
        Rotation2d currentAngle = getRotation2d();
        this.odometry = new DifferentialDriveOdometry(currentAngle);
        this.poseHistory =  new LimitedPoseMap(constants.historyLimit);
        this.invertGyro = constants.invertGyro;

        differentialDrive = new DifferentialDrive(left, right);
        configureMotors();
    }
    
    // create a required method for subclasses
    public abstract void configureMotors();

    // * CONTROL METHODS

    // drive with a given throttle and curve (arcade drive)
    public void arcadeDrive(double throttle, double curve) {
        differentialDrive.arcadeDrive(throttle, curve, false);
    }

    // drive with a given throttle for each side of the robot (tank drive)
    public void tankDrive(double leftThrottle, double rightThrottle) {
        differentialDrive.tankDrive(constants.flipLeft * leftThrottle, constants.flipRight * rightThrottle, false);
    }

    // a tank drive that sets the voltages of the motors
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        left.setVoltage(constants.flipLeft * leftVolts);
        right.setVoltage(constants.flipRight * rightVolts);
        differentialDrive.feed();
    }

    // a reversed version of tankdrivevolts
    public void tankDriveVoltsReverse(double leftVolts, double rightVolts) {
        this.tankDriveVolts(-leftVolts, -rightVolts);
    }

    // sets the speeds of the speedcontrollers rather than the differentrialdrive
    public void setThrottle(double speed) {
        left.set(constants.flipLeft * speed);
        right.set(constants.flipRight * speed);
        differentialDrive.feed();
    }
    
    // stops all motors
    public void stop() {
        setThrottle(0);
    }

    // * SENSOR methods

    // Raw encoder ticks from motors for position
    protected abstract double getRightPositionTicks(); 
    protected abstract double getLeftPositionTicks(); 
    
    // Raw encoder ticks from motors for velocity
    protected abstract double getRightVelocityTicksPerSecond(); 
    protected abstract double getLeftVelocityTicksPerSecond(); 

    /**
     * Converts any fundamental tick value into meters:
     * 
     * tick -> meters
     * ticks per second -> meters per second
     * ticks per second^2 -> meters per second
     * etc
     * 
     * @param rawTicks raw value read from encoders
     * @return value in meters
     */
    protected double getDistance(double rawTicks) {
        return rawTicks * constants.distancePerTick;
    }

    public double getRightPositionMeters() {
        return getDistance(getRightPositionTicks()) * constants.flipRight;
    }

    public double getLeftPositionMeters() {
        return getDistance(getLeftPositionTicks()) * constants.flipLeft;
    }

    public double getRightVelocityMetersPerSecond() {
        return getDistance(getRightVelocityTicksPerSecond()) * constants.flipRight;
    }

    public double getLeftVelocityMetersPerSecond() {
        return getDistance(getLeftVelocityTicksPerSecond()) * constants.flipLeft;
    }

    // * ODOMETRY METHODS
    
    /**
     * Sets odometry to {@link Pose2d} specified. Notes: the rotation remains as read from the AHRS.
     * 
     * @param pose position to set to
     */
    public void setOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, ahrs.getRotation2d());
    }

    /**
     * Sets odometry to (0, 0)
     */
    public void zeroOdometry() {
        setOdometry(new Pose2d());
    }

    /**
     * Resets the AHRS. 
     * 
     * ! Warning, this takes up to 10 seconds to complete (it recalibrates some stuff), use sparingly
     */
    public void resetAHRS() {
        ahrs.reset();
    }

    public Rotation2d getRotation2d() {
        return ahrs.getRotation2d();
    }

    public double getHeading() {
        return getRotation2d().getDegrees();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    // resets the positions of the encoders to 0
    protected abstract void resetEncoders();
    
    // * SUBSYSTEMBASE OVERRIDES

    @Override
    public void periodic() {
        // Update the pose
        odometry.update(getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

        // Log the pose
        poseHistory.put(Timer.getFPGATimestamp(), getPose());
    }
}
