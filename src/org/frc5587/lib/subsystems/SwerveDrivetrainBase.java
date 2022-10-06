package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrivetrainBase extends SubsystemBase {
    //modules are numbered left to right, front to back
    protected SwerveModuleBase module1;
    protected SwerveModuleBase module2;
    protected SwerveModuleBase module3;
    protected SwerveModuleBase module4;
    protected DriveConstants constants;

    public static class DriveConstants {
        // ROBOT LENGTH AND WIDTH ARE MEASURED AXLE TO AXLE
        public final double robotLength, robotWidth, maxVoltage;

        public DriveConstants(double robotLength, double robotWidth, double maxVoltage) {
            this.robotLength = robotLength;
            this.robotWidth = robotWidth;
            this.maxVoltage = maxVoltage;
        }
    }

    public SwerveDrivetrainBase(SwerveModuleBase module1, SwerveModuleBase module2, SwerveModuleBase module3, SwerveModuleBase module4, DriveConstants constants) {
        this.module1 = module1;
        this.module2 = module2;
        this.module3 = module3;
        this.module4 = module4;

        this.constants = constants;
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
}
