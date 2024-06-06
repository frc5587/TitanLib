package org.frc5587.lib.subsystems;

import org.frc5587.lib.subsystems.interfaces.GyroBase;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;

public class TitanPigeon2 extends GyroBase {
    private final Pigeon2 gyro;

    public TitanPigeon2(int canID) {
        this(canID, false);
    }

    public TitanPigeon2(int canID, String canBus) {
        this(canID, canBus, false);
    }

    public TitanPigeon2(int canID, boolean inverted) {
        super(inverted);
        this.gyro = new Pigeon2(canID);
    }

    public TitanPigeon2(int canID, String canBus, boolean inverted) {
        super(inverted);
        this.gyro = new Pigeon2(canID, canBus);
    }

    @Override
    public Rotation2d getRawRoll() {
        return Rotation2d.fromDegrees(gyro.getRoll().getValueAsDouble());
    }

    @Override
    public Rotation2d getRawPitch() {
        return Rotation2d.fromDegrees(gyro.getPitch().getValueAsDouble());
    }

    @Override
    public Rotation2d getRawYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble() / 2048);
    }

    @Override
    public Rotation2d getRawAccumulatedYaw() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pigeon2 getPigeon2() {
        return this.gyro;
    }
}
