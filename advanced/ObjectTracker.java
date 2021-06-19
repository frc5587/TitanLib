package org.frc5587.lib.advanced;

/**
 * This class can be used to track any object whether it be static or in motion
 * (in theory), however, as of 6/9/21 it was written with the intent of tracking
 * a stationary object (Infinite Recharge power port). Because the distance
 * gyros and accelerometers lose precisions as the robot moves both the robot
 * and object position should be updated as often as possible regardless of
 * whether its tracking a moving or stationary object.
 */
public class ObjectTracker {
    private double objX = Double.NaN; // meters
    private double objY = Double.NaN; // meters
    private double robotX = Double.NaN; // meters
    private double robotY = Double.NaN; // meters

    /**
     * Sets the robot position on the X/Y coordinate grid.
     * 
     * @param x left/right distance, generally refers to the horizontal direction
     *          (short-wise) on the field - METERS
     * @param y forward/backward distance, generally refers to the vertical
     *          direction (long-wise) on the field - METERS
     */
    public void setRobotPosition(double x, double y) {
        robotX = x;
        robotY = y;
    }

    /**
     * Sets the object position on the X/Y coordinate grid.
     * 
     * @param x left/right distance, generally refers to the horizontal direction
     *          (short-wise) on the field - METERS
     * @param y forward/backward distance, generally refers to the vertical
     *          direction (long-wise) on the field - METERS
     */
    public void setObjectPosition(double x, double y) {
        objX = x;
        objY = y;
    }

    /**
     * Sets the position of the object relative to the robot's current position.
     * This uses polar coordinates for simplicity
     * 
     * @param distance distance from the object - METERS
     * @param angle    angle the object makes with the starting heading of the
     *                 robot, which is generally straight downfield - RADIANS
     */
    public void setObjectRelativePosition(double distance, double angle) {
        setObjectPosition(robotX + (distance * Math.cos(angle)), robotY + (distance * Math.sin(angle)));
    }

    /**
     * Gets the angle the object is relative to the robot. Its basically the theta
     * of polar coordinates with the robot at the origin.
     * 
     * @return angle - RADIANS
     */
    public double getRelativeAngle() {
        return Math.atan2(getDiffY(), getDiffX()); // TODO: check if these have to be switched
    }

    /**
     * Gets the distance the object is from the robot. Its basically the r of polar
     * coordinates with the robot at the origin.
     * 
     * @return distance from object - METERS
     */
    public double getDistance() {
        return Math.sqrt(Math.pow(getDiffX(), 2) + Math.pow(getDiffY(), 2));
    }

    /**
     * Gets the difference in X values the object is from the robot. - is to the
     * 'left' and + is to the 'right'
     * 
     * @return difference in X - METERS
     */
    public double getDiffX() {
        return objX - robotX;
    }

    /**
     * Gets the difference in X values the object is from the robot. - is 'behind'
     * and + is 'forward'
     * 
     * @return difference in Y - METERS
     */
    public double getDiffY() {
        return objY - robotY;
    }

    /**
     * Checks if all the values have been initialize. This is done to allow lazy
     * loading of the various positions.
     * 
     * @return whether its ready
     */
    public boolean isReady() {
        if (objX != Double.NaN && objY != Double.NaN && robotX != Double.NaN && robotY != Double.NaN) {
            return true;
        } else {
            return false;
        }
    }

}
