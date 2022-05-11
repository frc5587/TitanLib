package org.frc5587.lib.auto;

import java.io.IOException;
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
        this(fileName, false);
    }

    /**
     * Used to load the Pathweaver trajectories into the {@link RamseteCommandWrapper}. 
     * Note: this expects the file to have the extension `.wpilib.json`
     * 
     * @param fileName name of file (everything before the extension)
     * @param pathPlannerDirectory whether the path is in the pathplanner directory. If true, the path will
     *                             be searched for in 'src/main/deploy/pathplanner/generatedJSON'. Otherwise
     *                             it will use the filepath 'src/main/deploy/paths/output'
     */
    public AutoPath(String fileName, boolean pathPlannerDirectory) {
        if(pathPlannerDirectory) {
            filePath = "pathplanner/generatedJSON/" + fileName + ".wpilib.json";
        } else {
            filePath = "paths/output/" + fileName + ".wpilib.json";
        }
        path = Filesystem.getDeployDirectory().toPath().resolve(filePath);

        try {
            this.trajectory = TrajectoryUtil.fromPathweaverJson(path);
        } catch (IOException e) {
            // Fail hard so people know quickly when something is wrong instead of a NullPointerException down the road
            throw new RuntimeException("Could not find trajectory file:   " + path);
        }
    }
}
