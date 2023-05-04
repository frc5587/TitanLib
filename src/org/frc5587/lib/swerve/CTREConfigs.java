package org.frc5587.lib.swerve;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

// TODO
public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConstants.ANGLE_LIMIT_ENABLED, 
            Constants.SwerveConstants.ANGLE_CONT_LIMIT, 
            Constants.SwerveConstants.ANGLE_PEAK_LIMIT, 
            Constants.SwerveConstants.ANGLE_PEAK_DURATION);

        swerveAngleFXConfig.slot0.kP = Constants.SwerveConstants.ANGLE_FPID.kP;
        swerveAngleFXConfig.slot0.kI = Constants.SwerveConstants.ANGLE_FPID.kI;
        swerveAngleFXConfig.slot0.kD = Constants.SwerveConstants.ANGLE_FPID.kD;
        swerveAngleFXConfig.slot0.kF = Constants.SwerveConstants.ANGLE_FPID.kF;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SwerveConstants.DRIVE_LIMIT_ENABLED, 
            Constants.SwerveConstants.DRIVE_CONT_LIMIT, 
            Constants.SwerveConstants.DRIVE_PEAK_LIMIT, 
            Constants.SwerveConstants.DRIVE_PEAK_DURATION);

        swerveDriveFXConfig.slot0.kP = Constants.SwerveConstants.DRIVE_FPID.kP;
        swerveDriveFXConfig.slot0.kI = Constants.SwerveConstants.DRIVE_FPID.kI;
        swerveDriveFXConfig.slot0.kD = Constants.SwerveConstants.DRIVE_FPID.kD;
        swerveDriveFXConfig.slot0.kF = Constants.SwerveConstants.DRIVE_FPID.kF;
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = Constants.SwerveConstants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = Constants.SwerveConstants.CLOSED_LOOP_RAMP;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = Constants.SwerveConstants.CANCODER_INVERTED;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}
