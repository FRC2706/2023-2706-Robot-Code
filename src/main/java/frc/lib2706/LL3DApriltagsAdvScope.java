// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib2706;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import frc.robot.config.Config;

/** Add your docs here. */
public class LL3DApriltagsAdvScope {
    // Pose to draw the green line to if loading AprilTagFieldLayout fails
    private final Pose3d ALTERNATIVE_POSE = new Pose3d(Config.FIELD_X/2, Config.FIELD_Y/2, 1, new Rotation3d());

    // The height that the blue/yellow cones should be placed at above the green ghost.
    private final double CONE_HEIGHT = 2.5;

    private DoubleArrayPublisher pubPose, pubAcceptedTarget, pubBlueFiltered, pubYellowFiltered;
    private AprilTagFieldLayout m_apriltagLayout;
    private boolean m_failedToLoadFieldLayout = false;

    public LL3DApriltagsAdvScope(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName + "3DAprilTagsAdvScope");
        pubPose = table.getDoubleArrayTopic("VisionRobotPose").publish(PubSubOption.periodic(0.02));
        pubAcceptedTarget = table.getDoubleArrayTopic("AcceptedTarget").publish(PubSubOption.periodic(0.02));
        pubBlueFiltered = table.getDoubleArrayTopic("BlueCone").publish(PubSubOption.periodic(0.02));
        pubYellowFiltered = table.getDoubleArrayTopic("YellowCone").publish(PubSubOption.periodic(0.02));
       
        try {
            m_apriltagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (Exception e) {
            m_failedToLoadFieldLayout = true;
        }   
    }

    public boolean acceptData(Pose2d pose, int tagId) {
        pubPose.accept(AdvantageUtil.deconstruct(pose));
        pubBlueFiltered.accept(new double[]{});
        pubYellowFiltered.accept(new double[]{});

        if (m_failedToLoadFieldLayout) {
            pubAcceptedTarget.accept(AdvantageUtil.deconstruct(ALTERNATIVE_POSE));
        } else {
            Optional<Pose3d> targetPose = m_apriltagLayout.getTagPose(tagId);
            if (targetPose.isEmpty()) {
                pubAcceptedTarget.accept(AdvantageUtil.deconstruct(ALTERNATIVE_POSE));
            } else {
                pubAcceptedTarget.accept(AdvantageUtil.deconstruct(targetPose.get()));
            }
        }

        return true;
    }

    public boolean rejectDataNoTargets() {
        pubPose.accept(new double[]{});
        pubAcceptedTarget.accept(new double[]{});
        pubBlueFiltered.accept(new double[]{});
        pubYellowFiltered.accept(new double[]{});

        return false;
    }

    public boolean rejectDataBlueFilter(Pose2d pose) {
        Pose3d bluePose = new Pose3d(
            pose.getX(),
            pose.getY(),
            CONE_HEIGHT,
            new Rotation3d(0.0, 0.0, pose.getRotation().getRadians())
        );

        pubPose.accept(AdvantageUtil.deconstruct(pose));
        pubAcceptedTarget.accept(new double[]{});
        pubBlueFiltered.accept(AdvantageUtil.deconstruct(bluePose));
        pubYellowFiltered.accept(new double[]{});
        
        return false;
    } 

    public boolean rejectDataYellowFilter(Pose2d pose) {
        Pose3d yellowPose = new Pose3d(
            pose.getX(),
            pose.getY(),
            CONE_HEIGHT,
            new Rotation3d(0.0, 0.0, pose.getRotation().getRadians())
        );

        pubPose.accept(AdvantageUtil.deconstruct(pose));
        pubAcceptedTarget.accept(new double[]{});
        pubBlueFiltered.accept(new double[]{});
        pubYellowFiltered.accept(AdvantageUtil.deconstruct(yellowPose));
        
        return false;
    } 
}