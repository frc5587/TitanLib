package org.frc5587.lib.subsystems;

import org.frc5587.lib.advanced.LimitedPoseMap;
import org.frc5587.lib.math.DriveSignal;
import org.frc5587.lib.math.Kinematics;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
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
    protected final LimitedPoseMap poseHistory;
    protected final DifferentialDriveKinematics kinematics;

    /**
    * A constants object that provides everything needed by {@link DrivetrainBase}
    */
    public static class DriveConstants {
        public final double wheelDiameterMeters, gearing, cpr, distancePerTick, trackWidth;
        public final int historyLimit;
        public final boolean invertGyro;

        /**
        * A constants object that provides everything needed by {@link DrivetrainBase}
        * @param wheelDiameterMeters the wheel diameter in meters
        * @param historyLimit   the limit of inputs for LimitedPoseMap
        * @param invertGyro     inverts values given by the gyroscope
        * @param cpr            the motor encoders' counts per revolution
        * @param gearing        the gearing from the motor output to the wheels
        * @param trackWidth     the distance between the left and right wheels in meters
        *                       (measured from the center of the width of the wheel)
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
        this.kinematics = new DifferentialDriveKinematics(constants.trackWidth);

        differentialDrive = new DifferentialDrive(left, right);
        differentialDrive.setDeadband(0);// Deadbanding is done in joystick
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
     * Based on 254's cheesyish drive from 2019.
     * 
     * @param throttle amount of power [-1, 1]
     * @param curve amount to turn [-1, 1]
     * @param quickTurn wether not to control the amount of curve with the throttle (slows curve down at low throttles)
     */
    public void titeDrive(double throttle, double curve, boolean quickTurn) {
        if (!quickTurn) {
            curve *= throttle;
        }

        // ! I'm 90% sure the math here won't work bc the units won't scale properly
        DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, curve), constants.trackWidth);
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));


        tankDrive(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor);
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
    * Recalibrates the AHRS.
    * <p>
    * !Warning! this takes up to 10 seconds to complete (it recalibrates some stuff), use sparingly
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

    /**
     * @return the angular velocity in radians
     */
    public double getAngularVelocity() {
        return Units.degreesToRadians(ahrs.getRate());
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
