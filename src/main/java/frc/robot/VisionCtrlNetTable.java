package frc.robot;

import edu.wpi.first.networktables.BooleanPublisher;
//for NetworkTable
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionCtrlNetTable {

    // For NetworkTables
    // Get the default instance of NetworkTables that was created automatically
    // when your program starts
    private NetworkTableInstance inst = NetworkTableInstance.getDefault();

    // Add a visionControl network table
    private NetworkTable visionControlTable;
    private BooleanPublisher VisionShutDownEntry;
    private BooleanPublisher VisionStartUpEntry;

    public VisionCtrlNetTable() {
        visionControlTable = inst.getTable("VisionControl");
        VisionShutDownEntry = visionControlTable.getBooleanTopic("ShutDown").publish();
        VisionStartUpEntry = visionControlTable.getBooleanTopic("StartUp").publish();

        VisionShutDownEntry.accept(false);
        VisionStartUpEntry.accept(false);
    }

    /**
     * Writes a shut down signal to the vision control table.
     *
     */
    public void shutDownVision() {
        VisionShutDownEntry.accept(true);
        VisionStartUpEntry.accept(false);

    }

    /**
     * Writes a start up signal to the vision control table.
     *
     */
    public void startUpVision() {
        VisionShutDownEntry.accept(false);
        VisionStartUpEntry.accept(true);
    }
}