package org.frc5587.lib.pathfinder;

import com.ctre.phoenix.motion.TrajectoryPoint;
import jaci.pathfinder.Trajectory;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;

import org.frc5587.lib.pathfinder.Pathgen;
import org.frc5587.lib.pathfinder.TalonPath;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class MotionProfileFiller extends CommandBase {
    AbstractDrive drive;
    Pathgen pathgen;
    String profileName;
    TrajectoryPoint[][] profiles;

    public MotionProfileFiller(AbstractDrive drive, Pathgen pathgen, String profileName, boolean zeroTalonDistance) {
        this.profileName = profileName;
        this.drive = drive;

        profiles = deserializeProfile(profileName, zeroTalonDistance);
    }

    public void initialize() {
        drive.queuePoints(profiles);
        fillBuffer(true);
    }

    public void execute() {
        drive.updateStatus();
    }

    public boolean isFinished() {
        boolean finished = drive.getStatuses()[0].isLast && drive.getStatuses()[1].isLast;
        return finished;
    }

    public TrajectoryPoint[][] deserializeProfile(String filename, boolean zeroTalonDistance) {
        System.out.println("Loading...");
        Trajectory unmodified = Pathgen.getTrajectoryFromFile(filename);
        Trajectory leftTrajectory = pathgen.getLeftSide(unmodified);
        Trajectory rightTrajectory = pathgen.getRightSide(unmodified);
        System.out.println("Trajectory Modified.");
        TrajectoryPoint[] leftTalonPoints = TalonPath.convertToTalon(leftTrajectory, 0, 0, drive.stuPerInch,
                zeroTalonDistance);
        TrajectoryPoint[] rightTalonPoints = TalonPath.convertToTalon(rightTrajectory, 0, 0, drive.stuPerInch,
                zeroTalonDistance);
        System.out.println("Converted to talon");
        debugTalonToFile(leftTalonPoints, "Left");
        debugTalonToFile(rightTalonPoints, "Right");

        return new TrajectoryPoint[][] { leftTalonPoints, rightTalonPoints };
    }

    public void fillBuffer(boolean enabled) {
        if (enabled) {
            drive.profileNotifer.startPeriodic(.005);
        } else {
            drive.profileNotifer.stop();
        }
    }

    private static void debugTalonToFile(TrajectoryPoint[] t, String fileNameAppended) {
        File myFile = new File("/home/lvuser/debugTalonTrajectory_" + fileNameAppended + ".csv");
        PrintWriter writer;
        try {
            writer = new PrintWriter(myFile);
            for (TrajectoryPoint pt : t) {
                writer.printf("%f,%f,%d,%b \n", pt.position, pt.velocity, pt.profileSlotSelect0, pt.isLastPoint);
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}