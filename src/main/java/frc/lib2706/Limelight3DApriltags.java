package frc.lib2706;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

public class Limelight3DApriltags {
    private final String limeLightName;

    private DoubleArrayPublisher pubAdvScopePose;

    public Limelight3DApriltags(String name) {
        limeLightName = name;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(name + "3DAprilTags");
        pubAdvScopePose = table.getDoubleArrayTopic("RobotPose").publish(PubSubOption.periodic(0.02));
    }

    public void update() {

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
