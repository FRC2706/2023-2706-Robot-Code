package frc.lib2706;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import frc.libLimelight.LimelightHelpers;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class LL3DApriltags {
    private final String m_limeLightName;
    private LL3DApriltagsAdvScope m_advScope;

    public LL3DApriltags(String name) {
        m_limeLightName = name;
        m_advScope = new LL3DApriltagsAdvScope(name);
    }

    public boolean update(boolean skipChecks) {
        if (!LimelightHelpers.getTV(m_limeLightName)) {
            return m_advScope.rejectDataNoTargets();
        }
        //seperate different alliances!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(m_limeLightName);
        int tagId = LimelightHelpers.getFiducialIDAsInt(m_limeLightName);
        Pose2d poseOdometry = SwerveSubsystem.getInstance().getPose();

        if (skipChecks == false) {
            // reject if the pose is off the field
            if (pose.getX() < 0 || pose.getX() > Config.FIELD_X || pose.getY() < 0 || pose.getY() > Config.FIELD_Y) {
                return m_advScope.rejectDataBlueFilter(pose);
            }

            //maybe later check for which end of the field vs tagId

            double extremeness = LimelightHelpers.getTLONG(m_limeLightName) / LimelightHelpers.getTSHORT(m_limeLightName);
            if (extremeness < 2) {
                // one tag
                Pose3d poseTag = LimelightHelpers.getTargetPose3d_CameraSpace(m_limeLightName);
                
                System.out.println("poseTag Hypo: " + Math.hypot(poseTag.getX(),poseTag.getZ()));

                if (Math.hypot(poseTag.getX(),poseTag.getZ()) > 2) {
                    
                    return m_advScope.rejectDataBlueFilter(pose);
                }

                if (poseOdometry.getX() > Config.FIELD_X * 1 / 5 && poseOdometry.getX() < Config.FIELD_X * 4 / 5) {
                    return m_advScope.rejectDataBlueFilter(pose);
                }

                if (Math.abs(poseOdometry.getX() - pose.getX()) > 0.5 || Math.abs(poseOdometry.getY() - pose.getY()) > 0.5 || Math.abs(pose.getRotation().getDegrees() - poseOdometry.getRotation().getDegrees()) > 10) {
                    return m_advScope.rejectDataBlueFilter(pose);
                }

                if (extremeness < 1.1) {
                    return m_advScope.rejectDataYellowFilter(pose);
                }

                
            } else {

                // more tags
                if (poseOdometry.getX() > Config.FIELD_X * 3 / 8 && poseOdometry.getX() < Config.FIELD_X * 5 / 8) {
                    return m_advScope.rejectDataBlueFilter(pose);
                }

                if (Math.abs(poseOdometry.getX() - pose.getX()) > 1 || Math.abs(poseOdometry.getY() - pose.getY()) > 1 || Math.abs(pose.getRotation().getDegrees() - poseOdometry.getRotation().getDegrees()) > 20) {
                    return m_advScope.rejectDataBlueFilter(pose);
                }
            }
        }

        double timestamp = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture(m_limeLightName) / 1000.0
                - LimelightHelpers.getLatency_Pipeline(m_limeLightName) / 1000.0;
        SwerveSubsystem.getInstance().addVisionMeasurement(pose, timestamp);
        return m_advScope.acceptData(pose, tagId);
    }
}
