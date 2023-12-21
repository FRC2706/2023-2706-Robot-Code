// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.photonSubsystem;

public class PhotonMoveToTarget extends CommandBase {

  Translation2d targetOffset;
  boolean centerTarget;
  Rotation2d desiredHeading;
  double tolerance=0.01;


  public PhotonMoveToTarget(Translation2d _targetOffset, double _tolerance) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(photonSubsystem.getInstance());
    targetOffset = _targetOffset;
    centerTarget=true;
    tolerance=_tolerance;
  }

  
  public PhotonMoveToTarget(Translation2d _targetOffset, Rotation2d _desiredHeading, double _tolerance) {
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(photonSubsystem.getInstance());
    targetOffset = _targetOffset;
    desiredHeading = _desiredHeading;
    centerTarget=false;
    tolerance=_tolerance;
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
    if (centerTarget){
      SwerveSubsystem.getInstance().driveToPose(new Pose2d(setPoint.plus(targetOffset), rotationSetPoint));
    }else{
      SwerveSubsystem.getInstance().driveToPose(new Pose2d(setPoint.plus(targetOffset), desiredHeading));
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // SwerveSubsystem.getInstance().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return SwerveSubsystem.getInstance().isAtPose(tolerance, tolerance*10);
  }
}
