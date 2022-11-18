package org.frc5587.lib.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveBase extends SubsystemBase {
    //modules are numbered left to right, front to back
    protected SwerveModuleBase module1;
    protected SwerveModuleBase module2;
    protected SwerveModuleBase module3;
    protected SwerveModuleBase module4;
    protected DriveConstants constants;

    // ! ODOMETRY
    protected AHRS ahrs = new AHRS();
    protected boolean invertGyro;
    protected SwerveDriveOdometry odometry;
    protected SwerveDrivePoseEstimator odometryEstimator;
    protected final TimeInterpolatableBuffer<Pose2d> poseHistory = TimeInterpolatableBuffer.createBuffer(1.5);
    protected final SwerveDriveKinematics kinematics;

    public static class DriveConstants {
        // ROBOT LENGTH AND WIDTH ARE MEASURED AXLE TO AXLE
        public final double robotLength, robotWidth, maxVoltage;
        public final boolean invertGyro;

        public DriveConstants(double robotLength, double robotWidth, double maxVoltage, boolean invertGyro) {
            this.robotLength = robotLength;
            this.robotWidth = robotWidth;
            this.maxVoltage = maxVoltage;
            this.invertGyro = invertGyro;
        }
    }

    public SwerveDriveBase(SwerveModuleBase module1, SwerveModuleBase module2, SwerveModuleBase module3, SwerveModuleBase module4, DriveConstants constants) {
        this.module1 = module1;
        this.module2 = module2;
        this.module3 = module3;
        this.module4 = module4;

        this.constants = constants;

        // TODO Check if these position calculations are accurate
        this.kinematics = new SwerveDriveKinematics(
            new Translation2d(constants.robotWidth / 2, -constants.robotLength / 2),
            new Translation2d(constants.robotWidth / 2, constants.robotLength / 2),
            new Translation2d(-constants.robotWidth / 2, -constants.robotLength / 2),
            new Translation2d(-constants.robotWidth / 2, constants.robotLength / 2)
        );
        this.odometry = new SwerveDriveOdometry(kinematics, getRotation2d());
        // TODO Check if these stds are accurate
        this.odometryEstimator = new SwerveDrivePoseEstimator(getRotation2d(), getPose(), kinematics,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.05, 0.05, Units.degreesToRadians(5)),
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(Units.degreesToRadians(0.1)),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, Units.degreesToRadians(0.1))
        );

        this.invertGyro = constants.invertGyro;
    }


    public void arcadeDrive(double x1, double y, double x2) {
        double r = Math.sqrt (Math.pow(constants.robotLength, 2) + Math.pow(constants.robotWidth, 2));
        y *= -1;

        double a = x1 - x2 * (constants.robotLength / r);
        double b = x1 + x2 * (constants.robotLength / r);
        double c = y - x2 * (constants.robotWidth / r);
        double d = y + x2 * (constants.robotWidth / r);

        double module1Voltage = Math.sqrt((b * b) + (c * c)) * constants.maxVoltage;
        double module2Voltage = Math.sqrt((b * b) + (d * d)) * constants.maxVoltage;
        double module3Voltage = Math.sqrt((a * a) + (c * c)) * constants.maxVoltage;
        double module4Voltage = Math.sqrt((a * a) + (d * d)) * constants.maxVoltage;

        double module1Angle = Math.atan2(b, c) / Math.PI;
        double module2Angle = Math.atan2(b, d) / Math.PI;
        double module3Angle = Math.atan2(a, c) / Math.PI;
        double module4Angle = Math.atan2(a, d) / Math.PI;

        module1.drive(module1Voltage, module1Angle);
        module2.drive(module2Voltage, module2Angle);
        module3.drive(module3Voltage, module3Angle);
        module4.drive(module4Voltage, module4Angle);
    }

    public Rotation2d getRotation2d() {
        return ahrs.getRotation2d();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    } 

    public void stop() {
        module1.stop();
        module2.stop();
        module3.stop();
        module4.stop();
    }
}
