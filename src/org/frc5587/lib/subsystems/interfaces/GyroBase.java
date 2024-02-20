package org.frc5587.lib.subsystems.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;

public abstract class GyroBase {
    protected enum Mode {
        SIGNED_NEG180_TO_180,
        UNSIGNED_0_TO_360
    }

    /**
     * A note on modes - internally, this class uses whichever mode is chosen in the
     * <code>mode</code> instance variable. All measurements supplied by the subclass
     * will be converted to the necessary mode <i>before</i> being returned by any methods in
     * this class.
     */
    protected Mode mode;
    protected Mode supplyMode;
    protected Rotation2d rollZeroOffset;
    protected Rotation2d pitchZeroOffset;
    protected Rotation2d yawZeroOffset;
    protected boolean inverted;

    /**
     * Creates a new Gyro.
     * @param rollZeroOffset an amount to subtract from the raw roll to reach an accurate zero position
     * @param pitchZeroOffset an amount to subtract from the raw pitch to reach an accurate zero position
     * @param yawZeroOffset an amount to subtract from the raw yaw to reach an accurate zero position
     * @param mode determines how to return the measurements, either signed (-180 to 180) or unsigned (0 to 360).
     * @param supplyMode determines how the subclasses will return the measurements and supply them to this class
     *                   through the raw measurement methods, either signed (-180 to 180) or unsigned (0 to 360).
     */
    public GyroBase(Rotation2d rollZeroOffset, Rotation2d pitchZeroOffset, Rotation2d yawZeroOffset, Mode mode, Mode supplyMode, boolean inverted) {
        this.rollZeroOffset = rollZeroOffset;
        this.pitchZeroOffset = pitchZeroOffset;
        this.yawZeroOffset = yawZeroOffset;
        this.mode = mode;
        this.supplyMode = supplyMode;
        this.inverted = inverted;
    }

    public GyroBase() {
        this(new Rotation2d(), new Rotation2d(), new Rotation2d(), Mode.SIGNED_NEG180_TO_180, Mode.SIGNED_NEG180_TO_180, false);
    }

    public GyroBase(boolean inverted) {
        this(new Rotation2d(), new Rotation2d(), new Rotation2d(), Mode.SIGNED_NEG180_TO_180, Mode.SIGNED_NEG180_TO_180, inverted);
    }

    public GyroBase(Mode mode, Mode supplyMode, boolean inverted) {
        this(new Rotation2d(), new Rotation2d(), new Rotation2d(), mode, supplyMode, inverted);
    }

    public GyroBase(Rotation2d rollZeroOffset, Rotation2d pitchZeroOffset, Rotation2d yawZeroOffset) {
        this(rollZeroOffset, pitchZeroOffset, yawZeroOffset, Mode.SIGNED_NEG180_TO_180, Mode.SIGNED_NEG180_TO_180, false);
    }
    
    public GyroBase(double rollZeroOffsetDegrees, double pitchZeroOffsetDegrees, double yawZeroOffsetDegrees, Mode mode, Mode supplyMode, boolean inverted) {
        this(Rotation2d.fromDegrees(rollZeroOffsetDegrees), Rotation2d.fromDegrees(pitchZeroOffsetDegrees), Rotation2d.fromDegrees(yawZeroOffsetDegrees), mode, supplyMode, inverted);
    }

    public GyroBase(double rollZeroOffsetDegrees, double pitchZeroOffsetDegrees, double yawZeroOffsetDegrees) {
        this(rollZeroOffsetDegrees, pitchZeroOffsetDegrees, yawZeroOffsetDegrees, Mode.SIGNED_NEG180_TO_180, Mode.SIGNED_NEG180_TO_180, false);
    }

    public Rotation2d convertMeasurementByMode(Rotation2d measurement) {
        if(mode.equals(supplyMode)) {
            return measurement;
        }
        else if(mode.equals(Mode.UNSIGNED_0_TO_360)) {
            return measurement.plus(Rotation2d.fromDegrees(180));
        }
        else if(mode.equals(Mode.SIGNED_NEG180_TO_180)) {
            return measurement.minus(Rotation2d.fromDegrees(180));
        }
        return measurement;
    }

    public Rotation2d getRollZeroOffset() {
        return this.rollZeroOffset;
    }

    public Rotation2d getPitchZeroOffset() {
        return this.pitchZeroOffset;
    }

    public Rotation2d getYawZeroOffset() {
        return this.yawZeroOffset;
    }

    public void setRollZeroOffset(Rotation2d zeroOffset) {
        this.rollZeroOffset = zeroOffset;
    }
    
    public void setPitchZeroOffset(Rotation2d zeroOffset) {
        this.pitchZeroOffset = zeroOffset;
    }

    public void setYawZeroOffset(Rotation2d zeroOffset) {
        this.yawZeroOffset = zeroOffset;
    }

    /**
     * @return the raw roll from the gyro, i.e. without subtracting the roll offset
     *         or applying the "Mode".
     */
    protected abstract Rotation2d getRawRoll();

    /**
     * @return the raw pitch from the gyro, i.e. without subtracting the pitch offset.
     *         or applying the "Mode".
     */
    protected abstract Rotation2d getRawPitch();

    /**
     * @return the raw yaw from the gyro, i.e. without subtracting the yaw offset.
     *         or applying the "Mode".
     */
    protected abstract Rotation2d getRawYaw();

    /**
     * @return the roll without subtracting the roll offset (but with any necessary mode conversion and inversion)
     */
    public Rotation2d getUnZeroedRoll() {
        return convertMeasurementByMode(getRawRoll()).times(this.inverted ? -1. : 1.);
    }
    
    /**
     * @return the pitch without subtracting the pitch offset (but with any necessary mode conversion and inversion)
     */
    public Rotation2d getUnZeroedPitch() {
        return convertMeasurementByMode(getRawPitch()).times(this.inverted ? -1. : 1.);
    }

    /**
     * @return the yaw without subtracting the yaw offset (but with any necessary mode conversion and inversion)
     */
    public Rotation2d getUnZeroedYaw() {
        return convertMeasurementByMode(getRawYaw()).times(this.inverted ? -1. : 1.);
    }

    /**
     * @return the accumulated yaw, i.e. the total angle the robot has gained since
     *         the last power cycle. This may go above 360 degrees.
     */
    public abstract Rotation2d getRawAccumulatedYaw();

    /**
     * Resets the zero position of the roll to the current roll of the gyro.
     */
    public void zeroRoll() {
        setRollZeroOffset(getUnZeroedRoll());
    }
    
    /**
     * Resets the zero position of the pitch to the current pitch of the gyro.
     */
    public void zeroPitch() {
        setPitchZeroOffset(getUnZeroedPitch());
    }

    /**
     * Resets the zero position of the yaw to the current yaw of the gyro.
     */
    public void zeroYaw() {
        setYawZeroOffset(getUnZeroedYaw());
    }

    public Rotation2d getRoll() {
        return getUnZeroedRoll().minus(rollZeroOffset);
    }

    public Rotation2d getPitch() {
        return getUnZeroedPitch().minus(pitchZeroOffset);
    }

    public Rotation2d getYaw() {
        return getUnZeroedYaw().minus(yawZeroOffset);
    }

    public Rotation3d getAllAs3d() {
        return new Rotation3d(getRoll().getRadians(), getPitch().getRadians(), getYaw().getRadians());
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }
    
    public boolean isInverted() {
        return this.inverted;
    }
}
