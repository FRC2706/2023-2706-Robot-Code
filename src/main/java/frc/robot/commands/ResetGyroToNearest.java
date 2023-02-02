// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;


import edu.wpi.first.wpilibj2.command.CommandBase;

public class ResetGyroToNearest extends CommandBase {
  /** Creates a new ResetGyroToNearest. */
  public ResetGyroToNearest() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = SwerveSubsystem.getInstance().getPose();
    double heading = SwerveSubsystem.getInstance().getHeading().getDegrees();
    heading = MathUtil.inputModulus(heading, 0, 360);
    if(heading > 315 && heading <= 45){
      Pose2d newPose = new Pose2d(pose.getTranslation(), new Rotation2d(Math.toRadians(0)));
      SwerveSubsystem.getInstance().resetOdometry(newPose);
    }
    else if(heading > 45 && heading <= 135){
      Pose2d newPose = new Pose2d(pose.getTranslation(), new Rotation2d(Math.toRadians(90)));
      SwerveSubsystem.getInstance().resetOdometry(newPose);
    }
    else if(heading > 135 && heading <= 225){
      Pose2d newPose = new Pose2d(pose.getTranslation(), new Rotation2d(Math.toRadians(180)));
      SwerveSubsystem.getInstance().resetOdometry(newPose);
    }
    else if(heading >225 && heading <=315){
      Pose2d newPose = new Pose2d(pose.getTranslation(), new Rotation2d(Math.toRadians(270)));
      SwerveSubsystem.getInstance().resetOdometry(newPose);
    }
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
    return false;
  }
}
