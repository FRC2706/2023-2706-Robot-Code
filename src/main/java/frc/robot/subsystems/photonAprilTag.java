// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class photonAprilTag extends SubsystemBase {
  private static photonAprilTag instance;
  private double EXAMPLE_SIZE_HEIGHT = 104.6;
  private double EXAMPLE_DISTANCE = 1.000;
  private Pose2d cameraOffset = new Pose2d(new Translation2d(0.23,0.3), Rotation2d.fromDegrees(0));
  //height of april = 1 foot 3 and 1/4 to bottom of black
  private DoubleArrayPublisher pubSetPoint;
  private PhotonCamera camera1;
  private Translation2d targetPos;
  private Rotation2d targetRotation;
  private LinearFilter filteryaw = LinearFilter.movingAverage(10);
  private LinearFilter filterX = LinearFilter.movingAverage(10);
  private LinearFilter filterY = LinearFilter.movingAverage(10);

  public static photonAprilTag getInstance(){
    if (instance == null){
      instance = new photonAprilTag();
    }
    return instance;
  }

  /** Creates a new photonAprilTag. */
  private photonAprilTag() {
    camera1 = new PhotonCamera("OV9281");
    pubSetPoint = NetworkTableInstance.getDefault().getTable(("PhotonCamera")).getDoubleArrayTopic("PhotonAprilPoint").publish(PubSubOption.periodic(0.02));
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
  }

  public void reset() {
    filterX.reset();
    filterY.reset();
    filteryaw.reset();
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
  }

  public Translation2d getTargetPos(){
    return targetPos;
  }

  public Rotation2d getTargetRotation(){
    return targetRotation;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera1.getLatestResult();
    
    if (result.hasTargets()){
      Pose2d odometryPose = SwerveSubsystem.getInstance().getPose();
      List<TargetCorner> corners = result.getBestTarget().getDetectedCorners();
      double heightSize = (corners.get(0).y - corners.get(3).y + corners.get(1).y - corners.get(2).y)/2;
      //System.out.println("tagsize "+heightSize);
      double range = EXAMPLE_SIZE_HEIGHT*EXAMPLE_DISTANCE/heightSize;
      //System.out.println("range"+range);
      Rotation2d yaw = Rotation2d.fromDegrees(-result.getBestTarget().getYaw());  

      Translation2d visionXY = new Translation2d(range, yaw);
      Translation2d robotRotated = visionXY.rotateBy(cameraOffset.getRotation());
      Translation2d robotToTargetRELATIVE = robotRotated.plus(cameraOffset.getTranslation());
      Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(odometryPose.getRotation());
      Translation2d feildToTarget = robotToTarget.plus(odometryPose.getTranslation());
      //System.out.println("robot to target "+ robotToTargetRELATIVE.toString());
      //System.out.println("odometry"+ odometryPose.toString());
      Rotation2d fieldOrientedTarget = yaw.rotateBy(odometryPose.getRotation());
      
      targetPos = new Translation2d(filterX.calculate(feildToTarget.getX())-1,filterY.calculate(feildToTarget.getY()));
      targetRotation = Rotation2d.fromDegrees(filteryaw.calculate(fieldOrientedTarget.getDegrees()));


      pubSetPoint.accept(new double[]{targetPos.getX(),targetPos.getY(),targetRotation.getRadians()});
    }
  }
}
