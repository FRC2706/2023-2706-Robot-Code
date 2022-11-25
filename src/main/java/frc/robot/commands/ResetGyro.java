// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetGyro extends CommandBase {
  
  /** 
   * ResetGyro will update the gyro offset in the {@link SwerveDriveOdometry}.
   * 
   * Keeps the X and Y of odometry the same.
   */
  public ResetGyro() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = DriveSubsystem.getInstance().getPose();
    Pose2d newPose = new Pose2d(pose.getTranslation(), new Rotation2d(0));
    DriveSubsystem.getInstance().resetOdometry(newPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
