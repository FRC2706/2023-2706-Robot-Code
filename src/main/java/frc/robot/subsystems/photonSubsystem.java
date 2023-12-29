// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//imports
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//class
public class photonSubsystem extends SubsystemBase {
  //declarations
  private static photonSubsystem instance;
  private DoubleArrayPublisher pubSetPoint;
  private DoublePublisher pubRange, pubYaw;
  private PhotonCamera camera1;
  private Translation2d targetPos;
  private Rotation2d targetRotation;
  private LinearFilter filteryaw = LinearFilter.movingAverage(10);
  private LinearFilter filterX = LinearFilter.movingAverage(10);
  private LinearFilter filterY = LinearFilter.movingAverage(10);
  private int data;
  private int id;

  //constants
  //height of april = 1 foot 3 and 1/4 to bottom of black from floor
  private double EXAMPLE_SIZE_HEIGHT = 104.6;
  private double EXAMPLE_DISTANCE = 1.000;
  private Pose2d cameraOffset = new Pose2d(new Translation2d(0.23,0.3), Rotation2d.fromDegrees(0));

  public static photonSubsystem getInstance(){
    if (instance == null){
      instance = new photonSubsystem();
    }
    return instance;
  }

  /** Creates a new photonAprilTag. */
  private photonSubsystem() {
    //name of camera, change if using multiple cameras
    camera1 = new PhotonCamera("OV9281");
    //networktable publishers
    pubSetPoint = NetworkTableInstance.getDefault().getTable(("PhotonCamera")).getDoubleArrayTopic("PhotonAprilPoint").publish(PubSubOption.periodic(0.02));
    pubRange = NetworkTableInstance.getDefault().getTable(("PhotonCamera")).getDoubleTopic("Range").publish(PubSubOption.periodic(0.02));
    pubYaw = NetworkTableInstance.getDefault().getTable(("PhotonCamera")).getDoubleTopic("Yaw").publish(PubSubOption.periodic(0.02));
    //set target info to the robot's info
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
    //initialize vars
    data = 0;
    id = -1;
  }

  public void reset(int desiredId) {
    filterX.reset();
    filterY.reset();
    filteryaw.reset();
    //set target info to the robot's info
    targetRotation = SwerveSubsystem.getInstance().getPose().getRotation();
    targetPos = SwerveSubsystem.getInstance().getPose().getTranslation();
    //initialize vars
    data = 0;
    id = desiredId;
  }

  public Translation2d getTargetPos(){
    return targetPos;
  }

  public Rotation2d getTargetRotation(){
    return targetRotation;
  }

  public boolean hasData() {
    //if data is the max that the filters hold
    return(data == 10);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result = camera1.getLatestResult();
    if (result.hasTargets()){
      //get the swerve pose at the time that the result was gotten
      Optional<Pose2d> optPose= SwerveSubsystem.getInstance().getPoseAtTimestamp(result.getTimestampSeconds());

      if (optPose.isEmpty()){
        return;
      }
      
      Pose2d odometryPose =optPose.get();
      PhotonTrackedTarget target = null;
      //currently chooses lowest id if sees two april tags
      if (id == -1){
        target = result.getBestTarget();
      } else{
        List<PhotonTrackedTarget> allTargets = result.getTargets();
        for (PhotonTrackedTarget t:allTargets){
          if (t.getFiducialId() == id){
           target = t;
           break;
          }
        }
        if (target == null){
          return;
        }
      }
      
      //calculate distance
      List<TargetCorner> corners = target.getDetectedCorners();
      double heightSize = (corners.get(0).y - corners.get(3).y + corners.get(1).y - corners.get(2).y)/2;
      double range = EXAMPLE_SIZE_HEIGHT*EXAMPLE_DISTANCE/heightSize;
      double targetx = 0;
      //calculating how far april tag is from center of camera field(not center of robot)
      for (int i = 0; i<4; i++){
        targetx += corners.get(i).x;
      }
      targetx = -(targetx/4-320)*35.0/320.0;

      Rotation2d yaw = Rotation2d.fromDegrees(targetx);  
      Rotation2d fieldOrientedTarget = yaw.rotateBy(odometryPose.getRotation());
      Translation2d visionXY = new Translation2d(range, yaw);
      Translation2d robotRotated = visionXY.rotateBy(cameraOffset.getRotation());
      Translation2d robotToTargetRELATIVE = robotRotated.plus(cameraOffset.getTranslation());
      Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(odometryPose.getRotation());
      Translation2d feildToTarget = robotToTarget.plus(odometryPose.getTranslation());
      
      targetPos = new Translation2d(filterX.calculate(feildToTarget.getX()),filterY.calculate(feildToTarget.getY()));
      targetRotation = Rotation2d.fromDegrees(filteryaw.calculate(fieldOrientedTarget.getDegrees()));
      data ++;

      pubSetPoint.accept(new double[]{targetPos.getX(),targetPos.getY(),targetRotation.getRadians()});
      pubRange.accept(range);
      pubYaw.accept(targetx);
    }
  }
}
