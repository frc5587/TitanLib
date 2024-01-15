package org.frc5587.lib.subsystems;

import org.frc5587.lib.subsystems.SwerveModuleBase.SwerveModuleConstants;
import org.frc5587.lib.subsystems.interfaces.GyroBase;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveBase extends SubsystemBase {
    public static class SwerveConstants {
        public final SwerveModuleConstants[] allModuleConstants;
        public final SwerveDriveKinematics kinematics;
        public final boolean gyroInverted;
        public final double maxSpeedMetersPerSecond;

        public SwerveConstants(SwerveModuleConstants[] allModuleConstants, SwerveDriveKinematics kinematics, boolean gyroInverted, double maxSpeedMetersPerSecond) {
            this.allModuleConstants = allModuleConstants;
            this.kinematics = kinematics;
            this.gyroInverted = gyroInverted;
            this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        }
    }

    public final SwerveConstants constants;
    public final SwerveModuleBase[] swerveModules;
    public final GyroBase gyro;
    
    public SwerveDriveOdometry odometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5); 
    
    public Field2d field = new Field2d();
    public Double lockedHeading = null;

    public SwerveBase(SwerveConstants constants, SwerveModuleBase[] swerveModules) {
        this.constants = constants;
        this.swerveModules = swerveModules;
        this.gyro = new TitanNavX(constants.gyroInverted);
        this.kinematics = constants.kinematics;

        zeroGyro();

        this.odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), getOdometryPose());
        
        Timer.delay(3.0);
        resetModulesToAbsolute();
    }

    /**
     * @param translation the amount of x-and-y-change to move the robot by - note that the measurement
     *                    given to this translation may not be the actual amount that the robot moves;
     *                    this is more like a proportional output (i.e. if the translation is 3 meters
     *                    by 4 meters, the robot may move 0.5 meters by 0.7 meters).
     * @param rotation the amount of rotation to move the robot by - note that the measurement given to
     *                 this rotation may not be the actual amount that the robot moves; this is more like
     *                 a proportional output (i.e. if the rotation is 50 degrees, the robot may move 12.5
     *                 degrees).
     * @param fieldRelative whether the input should be taken as field relative. For example: if true, a
     *                      positive x will move the robot away from the driver no matter the rotation of
     *                      the robot, and if false, a positive x will move the robot towards its own front
     *                      no matter the rotation of the robot, and so on.
     * @param isOpenLoop whether to use open-loop velocity control for the drive motors. If false, the velocity
     *                   of the robot may not be the same as the velocity calculated in the chassis speeds, but
     *                   it will probably be proportional.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(), 
                        translation.getY(), 
                        rotation, 
                        getYaw())
                    : new ChassisSpeeds(
                        translation.getX(),
                        translation.getY(), 
                        rotation));

        setModuleStates(swerveModuleStates, isOpenLoop);
    }    

    /**
     * Sets the desired states of each module.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.maxSpeedMetersPerSecond);
        
        for(SwerveModuleBase mod : swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleConstants.moduleNumber], isOpenLoop);
        }
    }

    /**
     * Sets the desired states of each module and assumes closed-loop control.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    }

    /**
     * Sets the module states based on chassis speeds.
     */
    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    /**
     * @return the robot pose from the pose estimator.
     */
    public Pose2d getPose() {
        return getEstimatedPose(); 
    }

    /**
     * @return the robot pose from odometry
     */
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    /**
     * @return the robot pose from the pose estimator.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets odometry, gyro yaw, and pose estimator to the given pose.
     * @param pose the position to reset the odometry to.
     */
    public void resetOdometry(Pose2d pose) {
        gyro.zeroYaw();
        gyro.setYawZeroOffset(pose.getRotation());

        odometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    /**
     * Resets odometry, gyro yaw, and pose estimator to an empty Pose2d
     * (i.e. 0 meters, 0 meters, 0 degrees)
     */
    public void zeroOdometry() {
        resetOdometry(new Pose2d());
    }

    /**
     * @return an array of SwerveModuleStates that represent the angle and velocity
     * of each module
     */
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleBase mod : swerveModules){
            states[mod.moduleConstants.moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * @return an array of SwerveModulePositions that represent the angle and drive motor
     * position of each module
     */
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModuleBase mod : swerveModules){
            positions[mod.moduleConstants.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    /**
     * Sets the zero position of the gyro's yaw to the current yaw position
     */
    public void zeroGyro(){
        lockedHeading = null;
        gyro.zeroYaw();
    }

    /**
     * @return the gyro roll.
     */
    public Rotation2d getRoll() {
        return gyro.getRoll();
    }

    /**
     * @return the gyro pitch.
     */
    public Rotation2d getPitch() {
        return gyro.getPitch();
    }

    /**
     * @return the gyro yaw.
     */
    public Rotation2d getYaw() {
        return gyro.getYaw();
    }

    /**
     * @return the yaw from odometry, notably does not include any gyro information
     */
    public Rotation2d getYawFromOdom() {
        return odometry.getPoseMeters().getRotation();
    }

    /**
     * Sets all angle motor encoder positions to the position of the absolute encoder
     */
    public void resetModulesToAbsolute(){
        for(SwerveModuleBase mod : swerveModules){
            mod.resetToAbsolute();
        }
    }

    /**
     * Sets the speed of all drive motors to 0 and sets the angle of all modules to their
     * current angle
     */
    public void stop() {
        for (SwerveModuleBase mod : swerveModules) {
            mod.stop();
        }
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePositions());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions());
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
        field.setRobotPose(getPose());
    }
}