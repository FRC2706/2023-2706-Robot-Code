// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class PoseEstimator {
    public static DoubleSubscriber distance;
    public static DoubleSubscriber yaw;
    public static DoubleSubscriber imageTimestamp;
    public static IntegerSubscriber aprilTagID;
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("DriveTrain");

    public static void update(){

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


    }
}