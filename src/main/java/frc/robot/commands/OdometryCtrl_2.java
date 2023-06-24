// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import java.util.ArrayList;
import java.util.List;



public class OdometryCtrl_2 extends CommandBase {
  /** Creates a new OdometryCtrl. */

  private BaseAutoBuilder m_autoBuilder;
  private SwerveSubsystem m_drive;
  private double m_maxVel;
  private double m_maxAccel;
  private List<PathPoint> m_pathPoints;
  private PathPoint m_finalPathPoint;

  private CommandBase m_activeCommand;

  public OdometryCtrl_2(BaseAutoBuilder autoBuilder, 
                       double maxVel, 
                       double maxAccel, 
                       PathPoint finalPathPoint) {

    m_autoBuilder = autoBuilder;
    m_maxVel = maxVel;
    m_maxAccel = maxAccel;
    m_finalPathPoint = finalPathPoint;
    
    m_pathPoints = new ArrayList<PathPoint>();
    m_pathPoints.add(0, null);
    m_pathPoints.add(1, null);
       
    m_drive = SwerveSubsystem.getInstance();
    addRequirements(m_drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = m_drive.getPose();
    ChassisSpeeds speeds = m_drive.getSpeedsFieldRelative();

    //set the first pathPoint: current (x,y) and current move moment
    m_pathPoints.set(0, new PathPoint(pose.getTranslation(), 
                                      new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond), 
                                      Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)));
   
   
    //
    //for testing only
    //overwrite the input final path point here: heading is 0, velocity is 0
    m_finalPathPoint = new PathPoint(pose.getTranslation().plus(new Translation2d(1,0)),
                            new Rotation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),//new Rotation2d(0), 
                                     0);


    m_pathPoints.set(1, m_finalPathPoint);
    //@todo: currently only two pathPoints. Later to add more waypoints

    PathPlannerTrajectory traj = PathPlanner.generatePath(
                                            new PathConstraints(m_maxVel, m_maxAccel), 
                                            m_pathPoints
                                            );


    m_activeCommand = m_autoBuilder.followPath(traj);

    m_activeCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_activeCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_activeCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_activeCommand.isFinished();
  }
}
