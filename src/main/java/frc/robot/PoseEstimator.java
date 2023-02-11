// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class PoseEstimator {
    public DoubleSubscriber distance;
    public DoubleSubscriber yaw;
    public DoubleSubscriber imageTimestamp;
    public IntegerSubscriber aprilTagID;
    public NetworkTable aprilTagTable;
    public double prevImageTimeStamp;
    public AprilTagFieldLayout aprilTagFieldLayout;
    public Rotation2d m_getHeading;
    public final Pose2d m_cameraPose;

    public PoseEstimator(){
        aprilTagTable = NetworkTableInstance.getDefault().getTable("AprilTagTable");
        distance = aprilTagTable.getDoubleTopic("Distance").subscribe(-99);
        yaw = aprilTagTable.getDoubleTopic("yaw").subscribe(-99);
        imageTimestamp = aprilTagTable.getDoubleTopic("Image Timestamp").subscribe(-99);
        aprilTagID = aprilTagTable.getIntegerTopic("April Tag ID").subscribe(-99); 
        m_getHeading = SwerveSubsystem.getInstance().getHeading();
        m_cameraPose = null;
    }

    public void update(){
        if(distance.get(-99) == -99){
            return;
        }
        if(yaw.get(-99) == -99){
            return;
        }
        if(imageTimestamp.get(-99) == -99){
            return;
        }
        if(aprilTagID.get(-99) == -99){
            return;
        }

        double currentImageTimeStamp = imageTimestamp.get();

        if(currentImageTimeStamp != prevImageTimeStamp){
            prevImageTimeStamp = currentImageTimeStamp;
        }


        Optional<Pose3d> fieldToTarget = aprilTagFieldLayout.getTagPose((int)aprilTagID.get());
        if (fieldToTarget.isPresent() == false) {
            // AprilTag ID is not in AprilTagFieldLayout
            return;
        }

        Pose2d newPoseEstimate = distAndYawToRobotPose(
                    distance,
                    yaw,
                    m_getHeading,
                    fieldToTarget.get(),
                    m_cameraPose);


        

        
                    
    }
}