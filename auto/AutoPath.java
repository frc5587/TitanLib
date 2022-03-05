package org.frc5587.lib.auto;

import java.io.File;
import java.io.IOException;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

public class AutoPath {
    private final String filePath;
    public final Path path;
    public Trajectory trajectory;

    /**
     * Used to load the Pathweaver trajectories into the {@link RamseteCommandWrapper}. 
     * Note: this expects the file to be in `src/main/deploy/paths/output/` and have the extension `.wpilib.json`
     * 
     * @param fileName name of file (everything before the extension)
     */
    public AutoPath(String fileName) {
        filePath = "paths/output/" + fileName + ".wpilib.json";
        path = Filesystem.getDeployDirectory().toPath().resolve(filePath);

        try {
            this.trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            // Fail hard so people know quickly when something is wrong instead of a NullPointerException down the road
            throw new RuntimeException("Could not find trajectory file:   " + path);
        }
    }
}
