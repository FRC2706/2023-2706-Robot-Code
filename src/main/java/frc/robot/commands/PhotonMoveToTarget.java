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
import frc.robot.subsystems.photonAprilTag;

public class PhotonMoveToTarget extends CommandBase {
  PIDController xController;
  PIDController yController;
  PIDController yawController;

  public PhotonMoveToTarget() {
    xController = new PIDController(0.5, 0.0, 0);
    yController = new PIDController(0.5, 0.0, 0);
    yawController = new PIDController(0.02, 0, 0);
    addRequirements(SwerveSubsystem.getInstance());
    addRequirements(photonAprilTag.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d odometryPose = SwerveSubsystem.getInstance().getPose();
    Translation2d setPoint = photonAprilTag.getInstance().getTargetPos();
    Rotation2d rotationSetPoint = photonAprilTag.getInstance().getTargetRotation();

    double xSpeed = xController.calculate(odometryPose.getX(), setPoint.getX());
    double yspeed = yController.calculate(odometryPose.getY(), setPoint.getY());
    double rotation = yawController.calculate(odometryPose.getRotation().getDegrees(), rotationSetPoint.getDegrees());

    SwerveSubsystem.getInstance().drive(xSpeed, yspeed, rotation, true, false);
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
