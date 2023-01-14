// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

/**
 * THIS IS A COPIED VERSION OF ({@link SwerveControllerCommand})
 * IT WAS MODIFIED TO ADD NETWORKTABLE VALUES
 * 
 * A command that uses two PID controllers ({@link PIDController}) and a ProfiledPIDController
 * ({@link ProfiledPIDController}) to follow a trajectory {@link Trajectory} with a swerve drive.
 *
 * <p>This command outputs the raw desired Swerve Module States ({@link SwerveModuleState}) in an
 * array. The desired wheel and module rotation velocities should be taken from those and used in
 * velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by the trajectory but rather goes
 * to the angle given in the final state of the trajectory.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
@SuppressWarnings("MemberName")
public class SwerveControllerCommandNetworkTables extends CommandBase {
  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final SwerveDriveKinematics m_kinematics;
  private final HolonomicDriveController m_controller;
  private final BiConsumer<SwerveModuleState[], Boolean> m_outputModuleStates;
  private final Supplier<Rotation2d> m_desiredRotation;

  // ADDED NETWORKTABLES
  NetworkTable table = NetworkTableInstance.getDefault().getTable("Auto");
  NetworkTableEntry xError = table.getEntry("xError");
  NetworkTableEntry yError = table.getEntry("yError");
  NetworkTableEntry rotError = table.getEntry("rotError");
  NetworkTableEntry rotSetpoint = table.getEntry("rotSetpoint");
  //
  
  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   *     time step.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("ParameterName")
  public SwerveControllerCommandNetworkTables(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      Supplier<Rotation2d> desiredRotation,
      BiConsumer<SwerveModuleState[], Boolean> outputModuleStates,
      Subsystem... requirements) {
    m_trajectory = trajectory;
    m_pose = pose;
    m_kinematics = kinematics;

    m_controller =
        new HolonomicDriveController(
            xController,
            yController,
            thetaController);

    m_outputModuleStates = outputModuleStates;

    m_desiredRotation = desiredRotation;

    addRequirements(requirements);
  }

  /**
   * Constructs a new SwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * <p>Note 2: The final rotation of the robot will be set to the rotation of the final pose in the
   * trajectory. The robot will not follow the rotations from the poses at each timestep. If
   * alternate rotation behavior is desired, the other constructor with a supplier for rotation
   * should be used.
   *
   * @param trajectory The trajectory to follow.
   * @param pose A function that supplies the robot pose - use one of the odometry classes to
   *     provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param thetaController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  @SuppressWarnings("ParameterName")
  public SwerveControllerCommandNetworkTables(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      SwerveDriveKinematics kinematics,
      PIDController xController,
      PIDController yController,
      ProfiledPIDController thetaController,
      BiConsumer<SwerveModuleState[], Boolean> outputModuleStates,
      Subsystem... requirements) {
    this(
        trajectory,
        pose,
        kinematics,
        xController,
        yController,
        thetaController,
        () ->
            trajectory.getStates().get(trajectory.getStates().size() - 1).poseMeters.getRotation(),
        outputModuleStates,
        requirements);
  }

  @Override
  public void initialize() {
    // ADDED NETWORKTABLES
    SwerveSubsystem.getInstance().setTrajectory(m_trajectory);
    
    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);

    // ADDED NETWORKTABLES
    Pose2d currentPose = m_pose.get();
    Pose2d poseError = desiredState.poseMeters.relativeTo(currentPose);
    xError.setDouble(poseError.getX());
    yError.setDouble(poseError.getY());

    Rotation2d desiredRotation = m_desiredRotation.get();
    rotSetpoint.setDouble(desiredRotation.getDegrees());
    rotError.setDouble(desiredRotation.getDegrees() - currentPose.getRotation().getDegrees());
    //

    var targetChassisSpeeds =
        m_controller.calculate(currentPose, desiredState, desiredRotation);

    var targetModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

    m_outputModuleStates.accept(targetModuleStates, false);
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}