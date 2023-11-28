// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.photonSubsystem;

public class PhotonMoveToTarget extends CommandBase {

  public PhotonMoveToTarget() {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(photonSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.getInstance().resetDriveToPose();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d setPoint = photonSubsystem.getInstance().getTargetPos();
    Rotation2d rotationSetPoint = photonSubsystem.getInstance().getTargetRotation();

    SwerveSubsystem.getInstance().driveToPose(new Pose2d(setPoint.plus(new Translation2d(-1,0)), rotationSetPoint));
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
