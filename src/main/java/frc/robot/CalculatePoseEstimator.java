// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public class CalculatePoseEstimator {
    public DoubleSubscriber distance;
    public DoubleSubscriber yaw;
    public DoubleSubscriber imageTimestamp;
    public IntegerSubscriber aprilTagID;
    public NetworkTable aprilTagTable;
    public double prevImageTimeStamp;
    public AprilTagFieldLayout aprilTagFieldLayout;
    public final Pose2d m_cameraPose;

    public CalculatePoseEstimator(){
        aprilTagTable = NetworkTableInstance.getDefault().getTable("AprilTagTable");
        distance = aprilTagTable.getDoubleTopic("Distance").subscribe(-99);
        yaw = aprilTagTable.getDoubleTopic("yaw").subscribe(-99);
        imageTimestamp = aprilTagTable.getDoubleTopic("Image Timestamp").subscribe(-99);
        aprilTagID = aprilTagTable.getIntegerTopic("April Tag ID").subscribe(-99);
        m_cameraPose = new Pose2d();
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
        else{
            return;
        }


        Optional<Pose3d> fieldToTarget = aprilTagFieldLayout.getTagPose((int)aprilTagID.get());

        if (fieldToTarget.isPresent() == false) {
            // AprilTag ID is not in AprilTagFieldLayout
            return;
        }

        Pose2d newPoseEstimate = distAndYawToRobotPose(
                    distance.get(),
                    yaw.get(),
                    SwerveSubsystem.getInstance().getHeading(),
                    fieldToTarget.get(),
                    m_cameraPose);

        if (newPoseEstimate.getX() < 0 || newPoseEstimate.getX() > Config.Swerve.fieldLength){
            return;
        }
        if (newPoseEstimate.getY() < 0 || newPoseEstimate.getY() > Config.Swerve.fieldWidth){
            return;
        }
        Pose2d pose = SwerveSubsystem.getInstance().getPose();
        double errorX = Math.abs(newPoseEstimate.getX() - pose.getX());
        double errorY = Math.abs(newPoseEstimate.getY() - pose.getY());

        if (errorX > Config.allowableVisionError){
            return;
        }
        if(errorY > Config.allowableVisionError){
            return;
        }

        SwerveSubsystem.getInstance().newVisionMeasurement(newPoseEstimate, Timer.getFPGATimestamp());
                    
    }

    /**
     * Calculate the Pose2d of the robot using vision data and the gyro.
     *
     * @param visionDistance Distance from the camera to the target in the camera's coordinate frame
     * @param visionYaw Yaw of the camera to the target in the camera's coordinate frame
     * @param heading Heading of the robot measured by the gyro
     * @param fieldToTarget Pose3d of the target in the field's coordinate frame. Use AprilTagFieldLayout.java for this.
     * @param robotToCamera Pose2d offset of the camera in the robot's coordinate frame
     *
     * @return Pose2d of the robot in the field's coordinate frame, as calculated from vision.
     */ 

    private static Pose2d distAndYawToRobotPose(double visionDistance, double visionYaw, Rotation2d heading, Pose3d fieldToTarget, Pose2d robotToCamera) {
        // Convert distance and yaw to XY
        Translation2d visionXY = new Translation2d(visionDistance, Rotation2d.fromDegrees(visionYaw));
        // Rotate XY from the camera's coordinate frame to the robot's coordinate frame
        Translation2d cameraToTargetRELATIVE = visionXY.rotateBy(robotToCamera.getRotation());
        // Add the XY from the robot center to camera and from the camera to the target
        Translation2d robotToTargetRELATIVE = robotToCamera.getTranslation().plus(cameraToTargetRELATIVE);
        // Rotate XY from the robot's coordinate frame to the field's coordinate frame and reverse the direction of the XY
        Translation2d targetToRobot = robotToTargetRELATIVE.rotateBy(
            heading
            ).unaryMinus();
        // Add the XY from the field to the target and from the target to the robot center
        Translation2d fieldToRobot = fieldToTarget.toPose2d().getTranslation().plus(targetToRobot);
        // Return the field to robot XY and set the gyro as the heading
        return new Pose2d(fieldToRobot, heading);
    } 
}