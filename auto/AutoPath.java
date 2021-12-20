package org.frc5587.lib.auto;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class AutoPath {
    private final String filePath;
    public final Path path;
    public Trajectory trajectory;

    public AutoPath(String fileName) {
        filePath = "paths/output/" + fileName + ".wpilib.json";
        path = Filesystem.getDeployDirectory().toPath().resolve(filePath);

        try {
            this.trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            System.out.println("*** WARNING***\nCould not find path file: " + filePath + "\n" + e + "\n--------");
            trajectory = null;  // If it throws an error here, its very annoying because then you have to catch it everywhere else
        }
    }
}