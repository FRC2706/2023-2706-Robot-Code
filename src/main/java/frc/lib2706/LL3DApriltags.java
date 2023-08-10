package frc.lib2706;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.libLimelight.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class LL3DApriltags {
    private final String m_limeLightName;
    private LL3DApriltagsAdvScope m_advScope;

    public LL3DApriltags(String name) {
        m_limeLightName = name;
        m_advScope = new LL3DApriltagsAdvScope(name);
    }

    public boolean update() {
        if (!LimelightHelpers.getTV(m_limeLightName)){
            return m_advScope.rejectDataNoTargets();
        }
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(m_limeLightName);
        int tagId = LimelightHelpers.getFiducialIDAsInt(m_limeLightName);

        // if (pose.getX()<0 || pose.getX() > Config.FIELD_X || pose.getY() < 0 || pose.getY() > Config.FIELD_Y){
        //     setAdvScopeNoPose();
        //     return;
        // }
        double extremeness = LimelightHelpers.getTLONG(m_limeLightName) / LimelightHelpers.getTSHORT(m_limeLightName);
        System.out.println(extremeness);
        if (extremeness < 1.15){
            return m_advScope.rejectDataBlueFilter(pose);
        }
        // Pose2d poseOdometry = SwerveSubsystem.getInstance().getPose();
        // if (poseOdometry.getX()>Config.FIELD_X*3/8 && poseOdometry.getX()<Config.FIELD_X*5/8){
        //     setAdvScopeNoPose();
        //     return;
        // }

        double timestamp = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture(m_limeLightName)/1000.0-LimelightHelpers.getLatency_Pipeline(m_limeLightName)/1000.0;
        SwerveSubsystem.getInstance().addVisionMeasurement(pose, timestamp);
        return m_advScope.acceptData(pose, tagId);
    }
}
