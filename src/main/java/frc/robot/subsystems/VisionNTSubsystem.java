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
public class VisionNTSubsystem extends SubsystemBase{
    public static VisionNTSubsystem instance;
    public final double ANGLE_ERROR_CODE = -99;
    public final double DISTANCE_ERROR_CODE = -1;
    public final double FLIP_Y = -1;
    public final double FLIP_X = 1;
    public final Pose2d TAPE_CAMERA_LOCATION = new Pose2d(-0.1, -0.1, Rotation2d.fromDegrees(180));
    public final double TIME_FOR_BAD_DATA = 1;

    private NetworkTable table = NetworkTableInstance.getDefault().getTable("MergeVisionPipelinePi21");
    private DoubleSubscriber tapeX;
    private DoubleSubscriber tapeY;

    private Timer tapeTimer = new Timer();

    private double calcTapeTargetX  = ANGLE_ERROR_CODE;
    private double calcTapeTargetY = ANGLE_ERROR_CODE;

    LinearFilter linearTapeFilterX = LinearFilter.movingAverage(10);
    LinearFilter linearTapeFilterY = LinearFilter.movingAverage(10);  

    private VisionNTSubsystem(){
        tapeX = table.getDoubleTopic("PoseY").subscribe(ANGLE_ERROR_CODE);
        tapeY = table.getDoubleTopic("PoseX").subscribe(ANGLE_ERROR_CODE);
        
        tapeTimer.restart();
    }

    public static VisionNTSubsystem getInstance(){
        if(instance == null){
            SubsystemChecker.subsystemConstructed(SubsystemType.VisionNTSubsystem);
            instance = new VisionNTSubsystem();
        }
        return instance;
    }

    public Translation2d calculateTapeTarget() {
        //Tape target
        if(tapeX.get() != ANGLE_ERROR_CODE || tapeY.get() != ANGLE_ERROR_CODE){
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
