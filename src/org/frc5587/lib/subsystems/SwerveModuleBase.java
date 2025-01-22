package org.frc5587.lib.subsystems;

import org.frc5587.lib.math.Conversions;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public abstract class SwerveModuleBase {
    public static class SwerveModuleConstants {
        public final int moduleNumber;
        public final double wheelCircumferenceMeters;
        public final double maxSpeedMetersPerSecond;
        public final int angleMotorEncoderCPR;
        public final int driveMotorEncoderCPR;
        public final double angleMotorGearRatio;
        public final double driveMotorGearRatio;

        /**
         * @param wheelCircumferenceMeters the wheel circumference in meters.
         * @param maxSpeedMetersPerSecond the maximum desired speed of the drive motors in m/s.
         * @param angleMotorEncoderCPR the counts per revolution of the angle motor's encoder.
         *                             SPARK MAXes already handle this value, so this value would
         *                             be 1 for swerves using SPARK MAX. CTRE motors with Talon FX
         *                             (Falcon 500, Kraken X60) have a CPR of 2048.
         * @param driveMotorEncoderCPR the counts per revolution of the drive motor's encoder.
         *                             SPARK MAXes already handle this value, so this value would
         *                             be 1 for swerves using SPARK MAX. CTRE motors with Talon FX
         *                             (Falcon 500, Kraken X60) have a CPR of 2048.
         * @param angleMotorGearRatio the gear ratio between the angle motor's output shaft and the
         *                            wheel's rotation. For SDS Mk4i modules, this would be 21.429.
         * @param driveMotorGearRatio the gear ratio between the drive motor's output shaft and the
         *                            wheel's rotation. For L1 SDS Mk4i modules, this would be 8.14.
         *                            L2 SDS Mk4i modules, this would be 6.75.
         */
        public SwerveModuleConstants(int moduleNumber, double wheelCircumferenceMeters, double maxSpeedMetersPerSecond, int angleMotorEncoderCPR, int driveMotorEncoderCPR, double angleMotorGearRatio, double driveMotorGearRatio) {
            this.moduleNumber = moduleNumber;
            this.wheelCircumferenceMeters = wheelCircumferenceMeters;
            this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
            this.angleMotorEncoderCPR = angleMotorEncoderCPR;
            this.driveMotorEncoderCPR = driveMotorEncoderCPR;
            this.angleMotorGearRatio = angleMotorGearRatio;
            this.driveMotorGearRatio = driveMotorGearRatio;
        }
    }

    protected Rotation2d lastAngle = new Rotation2d();
    protected TalonFX angleMotor;
    protected TalonFX driveMotor;
    protected SwerveModuleConstants moduleConstants;

    public SwerveModuleBase(SwerveModuleConstants moduleConstants, TalonFX angleMotor, TalonFX driveMotor) {
        this.moduleConstants = moduleConstants;
        
        // configureAngleEncoder();
        
        /* Angle Motor Config */
        this.angleMotor = angleMotor;
        // configureAngleMotor();

        /* Drive Motor Config */
        this.driveMotor = driveMotor;
        // configureDriveMotor();

        lastAngle = new Rotation2d();//getState().angle;
    }

    /**
     * Sets the entire module (angle motor and drive motor) to the desired state.
     * @param desiredState the SwerveModuleState to set the module to.
     * @param isOpenLoop whether to use open loop velocity control for the drive motor.
     *                   If this is true, the resulting velocity of the drive motor should
     *                   exactly match the supplied velocity from the SwerveModuleState (if
     *                   all control gains have been set properly).
     */
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = optimizeState(desiredState, getState().angle); 
        setAngle(desiredState);
        setDriveSpeed(desiredState, isOpenLoop);
    }

    /**
     * Sets the speed of the drive motor based on the desired state.
     * @param desiredState the SwerveModuleState to which supplies the velocity to set the
     *                     drive motor to.
     * @param isOpenLoop whether to use open loop velocity control for the drive motor.
     *                   If this is true, the resulting velocity of the drive motor should
     *                   exactly match the supplied velocity from the SwerveModuleState (if
     *                   all control gains have been set properly).
     */
    public void setDriveSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / moduleConstants.maxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        }
        else {
            setDriveMotorVelocity(desiredState.speedMetersPerSecond);
        }
    }

    /**
     * Sets the speed of the drive motor.
     * @param desiredState the velocity to set the drive motor to.
     * @param isOpenLoop whether to use open loop velocity control for the drive motor.
     *                   If this is true, the resulting velocity of the drive motor should
     *                   exactly match the supplied velocity from the SwerveModuleState (if
     *                   all control gains have been set properly). Otherwise, it will be
     *                   used as a percentage of the max speed.
     */
    public void setDriveSpeed(double speedMetersPerSecond, boolean isOpenLoop) {
        if(isOpenLoop){
            double percentOutput = speedMetersPerSecond / moduleConstants.maxSpeedMetersPerSecond;
            driveMotor.set(percentOutput);
        }
        else {
            setDriveMotorVelocity(speedMetersPerSecond);
        }
    }

    /**
     * Sets the position of the angle motor to the angle given by the desired state.
     * There is some optimization to decrease wheel jittering by preventing the module
     * from moving if the drive motor speed is less than 5% of the max speed.
     * @param desiredState the state that includes the drive motor's speed (necessary
     *                     for the optimization feature) and the desired angle.
     */
    public void setAngle(SwerveModuleState desiredState) {
        Rotation2d optimizedAngle = (Math.abs(desiredState.speedMetersPerSecond) <= (moduleConstants.maxSpeedMetersPerSecond * 0.05)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 5%. Prevents Jittering.
        setAngleMotorPosition(Conversions.mechanismOutputToMotorOutput(optimizedAngle, moduleConstants.angleMotorEncoderCPR, moduleConstants.angleMotorGearRatio));
        lastAngle = optimizedAngle;
    }

    /**
     * Sets the position of the angle motor to the given angle. Notably, this overload does
     * not include the speed optimization feature as there is no drive motor speed given to
     * the method.
     * @param angle the desired angle of the swerve module.
     */
    public void setAngle(Rotation2d angle) {        
        setAngleMotorPosition(angle);
        lastAngle = angle;
    }

    /**
     * @return the rotational position of the swerve module's wheel. In other words,
     *         the angle motor's position divided by its encoder CPR and its gear ratio
     *         relative to the wheel's rotation.
     */
    public Rotation2d getAngle(){
        return Conversions.motorOutputToMechanismOutput(getAngleMotorEncoderPosition(), moduleConstants.angleMotorEncoderCPR, moduleConstants.angleMotorGearRatio);
    }

    public abstract Rotation2d getAbsoluteEncoderValue();

    public void resetToAbsolute() {
        setAngleMotorEncoderPosition(Conversions.mechanismOutputToMotorOutput(getAbsoluteEncoderValue(), moduleConstants.angleMotorEncoderCPR, moduleConstants.angleMotorGearRatio));
    }

    protected abstract void setAngleMotorPosition(Rotation2d position);

    protected abstract void setAngleMotorEncoderPosition(Rotation2d position);

    protected abstract void setDriveMotorEncoderPosition(Rotation2d position);

    protected abstract void setDriveMotorVelocity(double velocityMPS);

    protected abstract void configureAngleEncoder();

    protected abstract void configureAngleMotor();

    protected abstract void configureDriveMotor();

    /**
     * @return the raw position value returned by the angle motor's built-in encoder.
     */
    protected abstract Rotation2d getAngleMotorEncoderPosition();

    /**
     * @return the raw position value returned by the drive motor's built-in encoder.
     */
    protected abstract Rotation2d getDriveMotorEncoderPosition();

    /**
     * @return the raw velocity value returned by the drive motor's built-in encoder.
     */
    protected abstract double getDriveMotorEncoderVelocity();

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimizeState(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            (getDriveMotorEncoderVelocity() / moduleConstants.driveMotorGearRatio) * moduleConstants.wheelCircumferenceMeters,
            getAngle()
        );
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.motorOutputToMeters(getDriveMotorEncoderPosition().times(1.), moduleConstants.driveMotorEncoderCPR, moduleConstants.driveMotorGearRatio, moduleConstants.wheelCircumferenceMeters),
            getAngle()
        );
    }

    /**
     * @param velocityMPS Velocity in meters/second.
     * @return Velocity in rotations/second.
     */
    public double mpsToRotationsPerSecond(double velocityMPS) {
        return ((velocityMPS) / (moduleConstants.wheelCircumferenceMeters * moduleConstants.driveMotorGearRatio));
    }


    public void stop() {
        setDesiredState(new SwerveModuleState(0, getAngle()), true);
    }
}