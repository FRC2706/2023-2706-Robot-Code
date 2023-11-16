// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import org.photonvision.*;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DiffTalonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class PhotonMoveToTarget extends CommandBase {
  double EXAMPLE_SIZE_HEIGHT = 104.6;
  double EXAMPLE_DISTANCE = 1.000;
  Pose2d cameraOffset = new Pose2d(new Translation2d(0.3,-0.3), Rotation2d.fromDegrees(0));
  //height of april = 1 foot 3 and 1/4 to bottom of black
  /** Creates a new ExampleCommand. */
  PhotonCamera camera1;
  PIDController xController;
  PIDController yController;
  PIDController yawController;
  Translation2d setPoint;
  Rotation2d rotationSetPoint;
  LinearFilter filteryaw = LinearFilter.movingAverage(5);
  LinearFilter filterX = LinearFilter.movingAverage(5);
  LinearFilter filterY = LinearFilter.movingAverage(5);


  public PhotonMoveToTarget() {
    camera1 = new PhotonCamera("OV9281");
    xController = new PIDController(1, 0.01, 0);
    yController = new PIDController(1, 0.01, 0);
    yawController = new PIDController(0.06, 0, 0.05);

    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var result = camera1.getLatestResult();
    Pose2d odometryPose = SwerveSubsystem.getInstance().getPose();
    if (result.hasTargets()){
      List<TargetCorner> corners = result.getBestTarget().getDetectedCorners();
      double heightSize = (corners.get(0).y - corners.get(3).y + corners.get(1).y - corners.get(2).y)/2;
      System.out.println("tagsize "+heightSize);
      double range = EXAMPLE_SIZE_HEIGHT*EXAMPLE_DISTANCE/heightSize;
      System.out.println("range"+range);
      Rotation2d yaw = Rotation2d.fromDegrees(-result.getBestTarget().getYaw());  
      

      Translation2d visionXY = new Translation2d(range, yaw);
      Translation2d robotToTargetRELATIVE = cameraOffset.getTranslation().plus(visionXY);
      Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(odometryPose.getRotation());
      Translation2d feildToTarget = robotToTarget.plus(odometryPose.getTranslation());
      System.out.println("robot to target "+ robotToTargetRELATIVE.toString());
      System.out.println("odometry"+ odometryPose.toString());
      Rotation2d fieldOrientedTarget = yaw.rotateBy(odometryPose.getRotation());
      
      setPoint = new Translation2d(filterX.calculate(feildToTarget.getX())-1,filterY.calculate(feildToTarget.getY()));
      rotationSetPoint = new Rotation2d(filteryaw.calculate(fieldOrientedTarget.getDegrees()));
    }
      System.out.println("set "+setPoint);
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
