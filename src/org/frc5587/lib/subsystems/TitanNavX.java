package org.frc5587.lib.subsystems;

import org.frc5587.lib.subsystems.interfaces.GyroBase;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;

public class TitanNavX extends GyroBase {
    private final AHRS gyro;

    public TitanNavX() {
        this.gyro = new AHRS();
    }

    public TitanNavX(boolean inverted) {
        super(inverted);
        this.gyro = new AHRS();
    }

    public TitanNavX(SPI.Port port) {
        this.gyro = new AHRS(port);
    }

    public TitanNavX(I2C.Port port) {
        this.gyro = new AHRS(port);
    }

    public TitanNavX(SerialPort.Port port) {
        this.gyro = new AHRS(port);
    }

    @Override
    public Rotation2d getRawAccumulatedYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    @Override
    public Rotation2d getRawRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll());
    }

    @Override
    public Rotation2d getRawPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch());
    }

    @Override
    public Rotation2d getRawYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public AHRS getNavX() {
        return this.gyro;
    }
}
