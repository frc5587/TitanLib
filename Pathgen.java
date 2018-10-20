package org.frc5587.lib;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import java.io.File;

public class Pathgen{
    double treadWidth, dt, vMax, aMax, jMax;
    Trajectory.Config config;
    public static final String DIRECTORY = "/home/lvuser/resources";


    /**
     * Use to configure the path planner
     * ALL UNITS ARE IN INCHES AND SECONDS
     * @param treadWidth Drivetrain wheel seperation (empirically
     * finding this may work better than just measuring)
     * @param dt Time between loops for profile to be generated on
     * @param vMax Max velocity of drivetrain
     * @param aMax Max acceleration of drivetrain
     * @param jMax Max jerk of drivetrain 
     */
    public Pathgen(double treadWidth, double dt, double vMax, double aMax, double jMax){
        this.treadWidth = treadWidth;
        this.dt = dt;
        this.vMax = vMax;
        this.aMax = aMax;
        this.jMax = jMax;
        this.config = new Trajectory.Config(
            Trajectory.FitMethod.HERMITE_CUBIC, 
            Trajectory.Config.SAMPLES_FAST, 
            this.dt,
            this.vMax, 
            this.aMax, 
            this.jMax
        );

    }

    /**
     * Generate profiles with this
     * ALL UNITS ARE IN INCHES
     * @param filename Where the file is written to
     * @param points Array of waypoints, Units are in inches and radians
     */
    public void createNew(String filename, Waypoint[] points){
        Trajectory trajectory = createTrajectory(points);
        System.out.println("Writing " + filename + " to csv");
        File resourcesDir = new File(DIRECTORY);
        if(!resourcesDir.exists()){
            resourcesDir.mkdirs();
        }
        File myFile = new File(DIRECTORY + "/" + filename + ".csv");
        Pathfinder.writeToCSV(myFile, trajectory);
    }

    /**
     * Returns the trajectory specific to the left side of the drivetrain
     */
    public Trajectory getLeftSide(Trajectory t){
        TankModifier modifier = new TankModifier(t).modify(treadWidth);
        return modifier.getLeftTrajectory();
    }

    /**
     * Returns the trajectory specific to the right side of the drivetrain
     */
    public Trajectory getRightSide(Trajectory t){
        TankModifier modifier = new TankModifier(t).modify(treadWidth);
        return modifier.getRightTrajectory();
    }
    
    /**
     * Load profile from file
     * @param name Filename of profile, don't append .csv extension
     */
    public static Trajectory getTrajectoryFromFile(String name){
        File myFile = new File(DIRECTORY + "/" + name + ".csv");
        Trajectory trajectory = Pathfinder.readFromCSV(myFile);
        return trajectory;
    }

    /**
     * Return the trajectory for a set of waypoints
     * @param points Array of waypoints, Units are in inches and radians
     */
    public Trajectory createTrajectory(Waypoint[] points) {
        return Pathfinder.generate(points, config);
    }

}
