package org.frc5587.lib;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motion.TrajectoryPoint.TrajectoryDuration;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

public class TalonPath{
/**
     * Converts a Pathfinder generated motion profile to one to send to a talon.
     * Assumes that this profile is the modified one for the respective side
     * of the drivetrain.
     * @param t The trajectory for THAT SPECIFIC SIDE of the drivetrain
     * @param currentPos current position of talon
     * @param pidfSlot what pidf slot to load into
     * @param nativeUnitsPerInch Conversion factor between inches and STU
     * @param resetTalonDistance will zero talons at start of profile if enabled
     */
    public static TrajectoryPoint[] convertToTalon(
        Trajectory t,
        double currentPos, 
        int pidfSlot, 
        double nativeUnitsPerInch,
        boolean resetTalonDistance
    ){
        if(resetTalonDistance){
            currentPos = 0;
        }
        int length = t.length();
        TrajectoryPoint[] outProfile = new TrajectoryPoint[length];
        for(int i = 0; i < length; i++){
            Segment s = t.get(i);
            TrajectoryPoint point = new TrajectoryPoint();
            point.profileSlotSelect0 = pidfSlot;
            point.position = (int)((s.position + currentPos)* nativeUnitsPerInch);
            point.velocity = (int)(s.velocity * nativeUnitsPerInch / 10);
            point.zeroPos = resetTalonDistance && i == 0;
            point.isLastPoint = (i+1) == length;
            point.timeDur = TrajectoryDuration.Trajectory_Duration_0ms;
            outProfile[i] = point;
        }
        return outProfile;
    }
}