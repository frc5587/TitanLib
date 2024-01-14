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

    public SwerveConstants constants;
    public SwerveModuleBase[] mSwerveMods;
    public GyroBase gyro;
    
    public SwerveDriveOdometry odometry;
    public SwerveDrivePoseEstimator poseEstimator;
    public SwerveDriveKinematics kinematics;
    public TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5); 
    
    public Field2d field = new Field2d();
    public Double lockedHeading = null;

    public SwerveBase() {
        this.gyro = new TitanNavX(constants.gyroInverted);
        this.kinematics = constants.kinematics;

        zeroGyro();

        this.odometry = new SwerveDriveOdometry(kinematics, getYaw(), getModulePositions());
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, getYaw(), getModulePositions(), getOdometryPose());
        
        Timer.delay(3.0);
        resetModulesToAbsolute();
    }

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

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, constants.maxSpeedMetersPerSecond);
        
        for(SwerveModuleBase mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }

        // setModuleStates(swerveModuleStates);
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, constants.maxSpeedMetersPerSecond);
        
        for(SwerveModuleBase mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        speeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public Pose2d getPose() {
        return getEstimatedPose(); 
    }

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        gyro.zeroYaw();
        gyro.setYawZeroOffset(pose.getRotation());

        odometry.resetPosition(getYaw(), getModulePositions(), pose);
        poseEstimator.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public void zeroOdometry() {
        resetOdometry(new Pose2d());
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModuleBase mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModuleBase mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        lockedHeading = null;
        gyro.zeroYaw();
    }

    public Rotation2d getRoll() {
        return gyro.getRoll();
    }

    public Rotation2d getPitch() {
        return gyro.getPitch();
    }

    public Rotation2d getYaw() {
        return gyro.getYaw();
    }

    public Rotation2d getYawFromOdom() {
        return odometry.getPoseMeters().getRotation();
    }

    public void resetModulesToAbsolute(){
        for(SwerveModuleBase mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public void stop() {
        for (SwerveModuleBase mod : mSwerveMods) {
            mod.stop();
        }
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePositions());
        poseEstimator.updateWithTime(Timer.getFPGATimestamp(), getYaw(), getModulePositions()); // ! If this is wrong, its probably a problem with getYaw()
        poseHistory.addSample(Timer.getFPGATimestamp(), getPose());
        field.setRobotPose(getPose());
        getModuleStates();
    }
}