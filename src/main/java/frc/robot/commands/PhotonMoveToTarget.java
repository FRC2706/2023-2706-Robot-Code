// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.*;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffTalonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonMoveToTarget extends CommandBase {
  double EXAMPLE_SIZE_HEIGHT = 25.808;
  double EXAMPLE_DISTANCE = 2.000;
  //height of april = 1 foot 3 and 1/4 to bottom of black
  /** Creates a new ExampleCommand. */
  PhotonCamera camera1;
  PIDController xController;
  PIDController yController;
  PIDController yawController;
  Translation2d setPoint;
  boolean seen = false;

  public PhotonMoveToTarget() {
    camera1 = new PhotonCamera("OV9281");
    xController = new PIDController(0.7, 0, 0);
    yController = new PIDController(0.7, 0, 0);
    yawController = new PIDController(0.06, 0, 5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seen = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera1.getLatestResult();
    Pose2d odometryPose = SwerveSubsystem.getInstance().getPose();
    if (result.hasTargets()){
      List<TargetCorner> corners = result.getBestTarget().getDetectedCorners();
      double heightSize = (corners.get(0).y - corners.get(3).y + corners.get(1).y - corners.get(2).y)/2;
      double range = heightSize/EXAMPLE_SIZE_HEIGHT*EXAMPLE_DISTANCE;
      double yaw = -result.getBestTarget().getYaw() + odometryPose.getRotation().getDegrees();

      double targetX = Math.cos(yaw)*range + odometryPose.getX()-1;
      System.out.println("range: " + range + " cos: " + Math.cos(yaw) + " Odometry: " + odometryPose.getX());
      double targetY = Math.sin(yaw)*range + odometryPose.getY();
      if (seen == false)
        setPoint = new Translation2d((targetX),(targetY));
      if (seen) 
        setPoint = new Translation2d((setPoint.getX()+targetX)/2,(setPoint.getY()+targetY)/2);
      seen = true;
      }
      if (seen) {
      double xSpeed = xController.calculate(odometryPose.getX(), setPoint.getX());
      double yspeed = yController.calculate(odometryPose.getY(), setPoint.getY());
      SwerveSubsystem.getInstance().drive(xSpeed, yspeed, 0, true, false);
      }
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
