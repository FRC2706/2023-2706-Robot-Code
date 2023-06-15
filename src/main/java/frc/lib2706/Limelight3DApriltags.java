package frc.lib2706;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import frc.libLimelight.LimelightHelpers;
import frc.robot.subsystems.DiffTalonSubsystem;

public class Limelight3DApriltags {
    private final String limeLightName;

    private DoubleArrayPublisher pubAdvScopePose;

    public Limelight3DApriltags(String name) {
        limeLightName = name;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(name + "3DAprilTags");
        pubAdvScopePose = table.getDoubleArrayTopic("RobotPose").publish(PubSubOption.periodic(0.02));
    }

    public void update() {
        if (!LimelightHelpers.getTV(limeLightName)){
            setAdvScopeNoPose();
            return;
        }
        Pose2d pose = LimelightHelpers.getBotPose2d_wpiBlue(limeLightName);
        setAdvScopePose(pose);
        double timestamp = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture(limeLightName)/1000.0-LimelightHelpers.getLatency_Pipeline(limeLightName)/1000.0;
        DiffTalonSubsystem.getInstance().newVisionMeasurement(pose,timestamp);
    }

    private void setAdvScopePose(Pose2d pose) {
        pubAdvScopePose.accept(AdvantageUtil.deconstruct(pose));
    }

    private void setAdvScopePose(Pose3d pose) {
        pubAdvScopePose.accept(AdvantageUtil.deconstruct(pose));
    }

    private void setAdvScopeNoPose() {
        pubAdvScopePose.accept(new double[]{});
    }
}
