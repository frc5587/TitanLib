package org.frc5587.lib.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;

public abstract class LimelightBase extends SubsystemBase {
    protected NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    protected NetworkTableEntry ty = limelightTable.getEntry("ty");
    protected NetworkTableEntry ledMode = limelightTable.getEntry("ledMode");
    protected double verticalOffset_Target = ty.getDouble(0.0);

    protected double mountAngle;
    protected double lensHeight;
    protected double goalHeight;

    /**
     * <a href="https://docs.limelightvision.io/en/latest/cs_estimating_distance.html#using-a-fixed-angle-camera">This</a> allows us to estimate distance from the Limelight to a target.
     * <p>
     * If the goal is roughly the same height as the Limelight, this equation is not useful.
     * @param mountAngle The degrees of difference from being perfectly perpendicular with the ground
     * @param lensHeight The distance from the ground to the center of the Limelight in meters
     * @param goalHeight The distance from the goal/target to the floor in meters
     * @see https://docs.limelightvision.io/en/latest/networktables_api.html
     */
    public LimelightBase(double mountAngle, double lensHeight, double goalHeight) {
        this.mountAngle = mountAngle;
        this.lensHeight = lensHeight;
        this.goalHeight = goalHeight;
    }

    /**
     * Calculate the angle to the goal in degrees
     * @return The angle to the goal in degrees
     */
    public double angleToGoalDegrees() {
        return mountAngle + verticalOffset_Target;
    }

    /**
     * Calculate the angle to the goal in radians using {@link #angleToGoalDegrees}
     * @return The angle to the goal in radians
     */
    public double angleToGoalRadians() {
        return Units.degreesToRadians(angleToGoalDegrees());
    }

    /**
     * Get the horizontal angle
     * @return 
     */
    public double getHorizontalAngle() {
        return limelightTable.getEntry("tx");
    }

    /**
     * Calculate the distance from the Limelight to the goal
     * @return The distance from the Limelight to goal in meters
     */
    public double calculateDistance() {
        return (goalHeight - lensHeight) / Math.tan(angleToGoalRadians());
    }

    /**
     * @return A boolean value Whether the Limelight detects a target
     */
    public boolean hasTarget() {
        return limelightTable.getEntry("tv");
    }

    /**
     * 
     */
    protected enum LedValues {
        DEFAULT,
        OFF,
        BLINK,
        ON
    }

    /**
     * Set the Limelight LEDs to a specific mode
     * @param value - DEFAULT, OFF, BLINK, ON
     */
    public void setLEDs(LedValues value) {
        switch(value) {
            case DEFAULT:
                ledMode.setNumber(0);
                break;
            case OFF:
                ledMode.setNumber(1);
                break;
            case BLINK:
                ledMode.setNumber(2);
                break;
            case ON:
                ledMode.setNumber(3);
                break;
            default:
                throw new RuntimeException("Invalid value (DEFAULT, OFF, BLINK, ON)");
        }
    }

    /**
     * Turn on Limelight LEDs
     */
    public void on() {
        setLEDs(3);
    }

    /**
     * Turn off Limelight LEDs
     */
    public void off() {
        setLEDs(1);
    }

    /**
     * Check if the Limelight LEDs are on
     * @return A truthy value if the Limelight LEDs are on
     */
    public boolean isOn() {
        return ledMode.getNumber() >= 1;
    }
}
