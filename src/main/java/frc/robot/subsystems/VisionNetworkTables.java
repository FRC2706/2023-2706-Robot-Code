// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;

/** Add your docs here. */
public class VisionNetworkTables extends SubsystemBase{
    public static VisionNetworkTables instance;
    public final double ANGLE_ERROR_CODE = -99;
    public final double DISTANCE_ERROR_CODE = -1;
    public final double FLIP_Y = -1;
    public final double FLIP_X = 1;
    public final Pose2d TAPE_CAMERA_LOCATION = new Pose2d(-0.1, -0.1, Rotation2d.fromDegrees(180));
    public final double TIME_FOR_BAD_DATA = 1;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("pipelineTape21");
    private DoubleSubscriber tapeX;
    private DoubleSubscriber tapeY;

    private Timer tapeTimer = new Timer();

    private double calcTapeTargetX  = DISTANCE_ERROR_CODE;
    private double calcTapeTargetY = DISTANCE_ERROR_CODE;

    LinearFilter linearTapeFilterX = LinearFilter.movingAverage(10);
    LinearFilter linearTapeFilterY = LinearFilter.movingAverage(10);  

    public VisionNetworkTables(){
        tapeX = table.getDoubleTopic("").subscribe(DISTANCE_ERROR_CODE);
        tapeY = table.getDoubleTopic("").subscribe(DISTANCE_ERROR_CODE);
        
        tapeTimer.restart();
    }

    public static VisionNetworkTables getInstance(){
        if(instance == null){
            SubsystemChecker.subsystemConstructed(SubsystemType.VisionNetworkTables);
            instance = new VisionNetworkTables();
        }
        return instance;
    }

    public Translation2d calculateTapeTarget() {
        //Tape target
        if(tapeX.get() != DISTANCE_ERROR_CODE || tapeY.get() != DISTANCE_ERROR_CODE){
            double visionDistanceX = Units.feetToMeters(tapeX.getAsDouble()) * FLIP_X;
            double visionDistanceY = Units.feetToMeters(tapeY.getAsDouble()) * FLIP_Y;

            Transform2d visionVector = new Transform2d(new Translation2d(visionDistanceX, visionDistanceY), new Rotation2d(0));

            Pose2d robotRelative = TAPE_CAMERA_LOCATION.plus(visionVector);

            Pose2d pose = SwerveSubsystem.getInstance().getPose();

            Pose2d fieldRelativeTarget = pose.plus(new Transform2d(new Pose2d(), robotRelative));

            calcTapeTargetX = linearTapeFilterX.calculate(fieldRelativeTarget.getX());
            calcTapeTargetY = linearTapeFilterY.calculate(fieldRelativeTarget.getY());

            tapeTimer.reset();
            return new Translation2d(calcTapeTargetX, calcTapeTargetY);
        }
        else if(tapeTimer.hasElapsed(1)){
            linearTapeFilterX.reset();
            linearTapeFilterY.reset();
        }
        return null;
    }
}
