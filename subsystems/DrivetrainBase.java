package org.frc5587.lib.subsystems;

import org.frc5587.lib.advanced.LimitedPoseMap;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;

public abstract class DrivetrainBase extends SubsystemBase {
    /** create leader and follower motors for the drivetrain */
    protected MotorController left;
    protected MotorController right;

    /** make the speed controller groups into one drivetrain object */
    protected DifferentialDrive differentialDrive;

    /** create a hashtable to look up constants */
    protected DriveConstants constants;

    /** create variables needed for odometry. */
    protected AHRS ahrs = new AHRS();
    protected boolean invertGyro;
    protected DifferentialDriveOdometry odometry;
    private final LimitedPoseMap poseHistory;

    public static class DriveConstants {
        public final double wheelDiameterMeters, gearing, cpr, distancePerTick;
        public final int historyLimit;
        public final boolean invertGyro;

        /**
        * A constants object that provides everything needed by {@link DrivetrainBase}
        * @param wheelDiameterMeters the wheel diameter in meters
        * @param historyLimit   the limit of inputs for LimitedPoseMap
        * @param invertGyro     inverts values given by the gyroscope
        * @param cpr            the motor encoders' counts per revolution
        * @param gearing        the gearing from the motor output to the wheels
        */
        public DriveConstants(double wheelDiameterMeters, int historyLimit, boolean invertGyro, double cpr,
                double gearing) {
            this.wheelDiameterMeters = wheelDiameterMeters;
            this.historyLimit = historyLimit;
            this.invertGyro = invertGyro;
            this.cpr = cpr;
            this.gearing = gearing;
            this.distancePerTick = ((1.0 / cpr) / gearing) * (Math.PI * wheelDiameterMeters);
        }
    }

    /**
    * A drivetrain that uses:
    * @param left        a MotorController for the left side of the drivetrain 
    *                       (can be passed as a MotorController Group)
    * @param right      a MotorController for the right side of the drivetrain 
    *                       (can be passed as a MotorController Group)
    * @param constants  a {@link DriveConstants} object containing all constants used by the class
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
        this.poseHistory = new LimitedPoseMap(constants.historyLimit);
        this.invertGyro = constants.invertGyro;

        differentialDrive = new DifferentialDrive(left, right);
        configureMotors();
    }

    /**
    * The implementing class must configure the motors <p>
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
    * drive with a given throttle for each side of the robot (tank drive)
    */ 
    public void tankDrive(double leftThrottle, double rightThrottle) {
        differentialDrive.tankDrive(
            leftThrottle,
            rightThrottle,
            false
        );
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
    *  an inverted version of tankdrivevolts
    */
    public void tankDriveVoltsReverse(double leftVolts, double rightVolts) {
        this.tankDriveVolts(-leftVolts, -rightVolts);
    }

    /** 
    * sets the speeds of the MotorControllers rather than using the DifferentrialDrive
    */
    public void setThrottle(double speed) {
        left.set(speed);
        right.set(speed);
        differentialDrive.feed();
    }

    // stops all motors
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
    * @return The position in velocity in meters per second, converted from encoder ticks
    */
    public double getRightVelocityMetersPerSecond() {
        return getDistance(getRightVelocityTicksPerSecond());
    }

    /**
    * @return The position in velocity in meters per second, converted from encoder ticks
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
    * <p>
    * !Warning! this takes up to 10 seconds to complete (it recalibrates some stuff), use sparingly
    */
    public void resetAHRS() {
        ahrs.reset();
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

    /**
    * @return the velocity of both sides of the drivetrain.
    */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
    }

    /**
    * @return The position of the robot on the field (meters)
    */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
    * resets the positions of the encoders to 0
    */
    protected abstract void resetEncoders();

    @Override
    public void periodic() {
        // Update the pose
        odometry.update(getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

        // Log the pose
        poseHistory.put(Timer.getFPGATimestamp(), getPose());
    }
}
