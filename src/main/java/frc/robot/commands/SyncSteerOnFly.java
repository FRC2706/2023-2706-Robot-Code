// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class SyncSteerOnFly extends CommandBase {
  /** Creates a new SyncSteerOnFly. */
  public SyncSteerOnFly() {
    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d pose = SwerveSubsystem.getInstance().getPose();
    SwerveSubsystem.getInstance().resetEncodersFromCanCoder();
    SwerveSubsystem.getInstance().resetLastAngles();
    SwerveSubsystem.getInstance().resetOdometry(pose);
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
