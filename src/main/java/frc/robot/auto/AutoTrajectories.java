// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.io.IOException;
import java.nio.file.InvalidPathException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.auto.AutoSelector.AutoRoutine;

/** Add your docs here. */
public class AutoTrajectories {
    private static final String PATHWEAVER_FOLDER = "PathWeaver//output";

    private static HashMap<AutoRoutine, Trajectory[]> trajectories = new HashMap<>();

    public static void constructTrajectories() {

        // Test1
        trajectories.put(AutoRoutine.Test1, new Trajectory[]{
            pathWeaver("Test1R1")
        });

        // Test2
        trajectories.put(AutoRoutine.Test2, new Trajectory[]{
            pathWeaver("Test2R1")
        });

        // Test3
        trajectories.put(AutoRoutine.Test3, new Trajectory[]{
            pathWeaver("Test3R1")
        });

        // Test4
        trajectories.put(AutoRoutine.Test4, new Trajectory[]{
            pathWeaver("Test4R1")
        });

        // Test5
        trajectories.put(AutoRoutine.Test5, new Trajectory[]{
            pathWeaver("Test5R1")
        });

        // RapidReactTest
        trajectories.put(AutoRoutine.RapidReactTest, new Trajectory[]{
            pathWeaver("RapidReactTestR1"),
            pathWeaver("RapidReactTestR2"),
            pathWeaver("RapidReactTestR3")
        });
    }


    public static Trajectory[] getTrajectories(AutoRoutine routine) {
        if (trajectories.containsKey(routine)) {
            Trajectory[] trajs = trajectories.get(routine);
            
            for (Trajectory traj : trajs) {
                if (traj == null) {
                    return null;
                }
            }
            return trajs;
        }
        return null;
    }

    private static Trajectory pathWeaver(String filename) {
        Path path;
        try {
            path = Filesystem.getDeployDirectory().toPath().resolve(PATHWEAVER_FOLDER + "//" + filename + ".wpilib.json");

        } catch (InvalidPathException e) {
            DriverStation.reportError("Failed to get path of the Trajectory named: " + filename, e.getStackTrace());
            return null;
        }

        try {
            return TrajectoryUtil.fromPathweaverJson(path);

        } catch (IOException e) {
            DriverStation.reportError("Failed construct Trajectory from PathWeaver JSON file. Trajectory named: " + filename, e.getStackTrace());
            return null;
        }
        
    }
}
