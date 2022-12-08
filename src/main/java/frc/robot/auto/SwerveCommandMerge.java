// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommandMerge extends SwerveControllerCommandNetworkTables {
    static private Supplier<SwerveSubsystem> sub = SwerveSubsystem::getInstance;  

    /**
     *  SwerveCommandMerge simplfies the construction of SwerveControllerCommands.
     * 
     *  Creates a new SwerveControllerCommand which will ask the desiredRotation supplier
     *  for the desired heading at that point in time.
     * 
     *  Go to SwerveControllerCommand for more details.
     */
    public SwerveCommandMerge(Trajectory trajectory, Supplier<Rotation2d> desiredRotation) { 
        super(
            trajectory,
            sub.get()::getPose,
            Config.Swerve.kSwerveDriveKinematics,
            new PIDController(Config.Swerve.kPXController, 0, 0),
            new PIDController(Config.Swerve.kPYController, 0, 0),
            getNewThetaController(), 
            desiredRotation,
            sub.get()::setModuleStates,
            sub.get()
        );
    }


    /**
     *  Creates a new SwerveControllerCommand that keeps the heading locked in as 
     *  the final heading of the trajectory (like tank drive / Ramsete)
     *  (rotates to that heading and stays at that heading for the remainder of the path)
     * 
     *  Go to SwerveControllerCommand for more details.
     */
    public SwerveCommandMerge(Trajectory trajectory) {
        super(
            trajectory,
            sub.get()::getPose,
            Config.Swerve.kSwerveDriveKinematics,
            new PIDController(Config.Swerve.kPXController, 0, 0),
            new PIDController(Config.Swerve.kPYController, 0, 0),
            getNewThetaController(), 
            sub.get()::setModuleStates,
            sub.get()
        );
    }

    /**
     *  Creates a new SwerveControllerCommand that keeps the heading locked in as 
     *  the given heading. 
     *  (rotates to that heading and stays at that heading for the remainder of the path)
     * 
     *  Go to SwerveControllerCommand for more details.
     */
    public SwerveCommandMerge(Trajectory trajectory, Rotation2d finalHeading) {
        this(trajectory, () -> passAngle(finalHeading));
    }

    /**
     * Helper function to make this class possible.
     * 
     * In order for the above constructors to work they must be in a static context. 
     * But the "enabledContinousInput" must be done in a non-static context. 
     * So this function will deal with the static/non-static.
     * 
     * @return A new ProfiledPidController with constants from Config.java 
     */
    private static ProfiledPIDController getNewThetaController() {
        ProfiledPIDController thetaController = new ProfiledPIDController(
            Config.Swerve.kPThetaController, 0, 0,    // P, I, D
            Config.Swerve.kThetaControllerConstraints);  // Trapizoid profile

            
        // Allows the robot to allows go the shorter direction around the circle to the target.
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return thetaController;
    }

    /**
     * SwerveControllerCommand demands a suppler but all we want to do in pass a single Rotation2d/
     * This function is just being that supplier.
     */
    private static Rotation2d passAngle(Rotation2d angle) {
        return angle;
    }
}
