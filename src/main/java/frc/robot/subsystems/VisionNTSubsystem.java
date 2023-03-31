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
    public final double TAPE_FLIP_Y = -1;
    public final double TAPE_FLIP_X = 1;
    public final double APRIL_FLIP_Y = -1;
    public final double APRIL_FLIP_X = 1;
    public final Pose2d TAPE_CAMERA_LOCATION = new Pose2d(Units.inchesToMeters(7.5), Units.inchesToMeters(10), Rotation2d.fromDegrees(0));
    public final Pose2d APRIL_CAMERA_LOCATION = new Pose2d(Units.inchesToMeters(7.5), Units.inchesToMeters(-10), Rotation2d.fromDegrees(0));
    public final double TIME_FOR_BAD_DATA = 1;

    // Tape
    private NetworkTable tapeTable = NetworkTableInstance.getDefault().getTable("MergeVisionPipelineTape21");
    private DoubleSubscriber tapeX;
    private DoubleSubscriber tapeY;

    private Timer tapeTimer = new Timer();

    private double calcTapeTargetX  = ANGLE_ERROR_CODE;
    private double calcTapeTargetY = ANGLE_ERROR_CODE;

    LinearFilter linearTapeFilterX = LinearFilter.movingAverage(10);
    LinearFilter linearTapeFilterY = LinearFilter.movingAverage(10);

    // Apriltag
    private NetworkTable aprilTable = NetworkTableInstance.getDefault().getTable("MergeVisionPipelineAprilTag23");
    private DoubleSubscriber aprilX;
    private DoubleSubscriber aprilY;
    private DoubleSubscriber aprilTimestamp;

    private Timer aprilTimer = new Timer();

    private double calcAprilTargetX  = ANGLE_ERROR_CODE;
    private double calcAprilTargetY = ANGLE_ERROR_CODE;

    LinearFilter linearAprilFilterX = LinearFilter.movingAverage(10);
    LinearFilter linearAprilFilterY = LinearFilter.movingAverage(10); 


    private VisionNTSubsystem(){
        tapeX = tapeTable.getDoubleTopic("PoseZ").subscribe(ANGLE_ERROR_CODE);
        tapeY = tapeTable.getDoubleTopic("PoseX").subscribe(ANGLE_ERROR_CODE);

        aprilX = aprilTable.getDoubleTopic("PoseX").subscribe(ANGLE_ERROR_CODE);
        aprilY = aprilTable.getDoubleTopic("PoseZ").subscribe(ANGLE_ERROR_CODE);
        aprilTimestamp = aprilTable.getDoubleTopic("TimeSincelastPublish").subscribe(ANGLE_ERROR_CODE);
        tapeTimer.restart();
        aprilTimer.restart();
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
        if(tapeX.get() != ANGLE_ERROR_CODE && tapeY.get() != ANGLE_ERROR_CODE){
            double visionDistanceX = Units.inchesToMeters(tapeX.getAsDouble()) * TAPE_FLIP_X;
            double visionDistanceY = Units.inchesToMeters(tapeY.getAsDouble()) * TAPE_FLIP_Y;

            // Transform2d visionVector = new Transform2d(new Translation2d(visionDistanceX, visionDistanceY), new Rotation2d(0));

            // Pose2d robotRelative = TAPE_CAMERA_LOCATION.plus(visionVector);

            // Pose2d pose = SwerveSubsystem.getInstance().getPose();

            // Pose2d fieldRelativeTarget = pose.plus(new Transform2d(new Pose2d(), robotRelative));
            
            Translation2d fieldRelativeTarget = distAndYawToRobotPose(visionDistanceX, visionDistanceY, SwerveSubsystem.getInstance().getHeading(), TAPE_CAMERA_LOCATION).getTranslation();

            // System.out.println(fieldRelativeTarget);
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

            public Translation2d calculateAprilTarget() {
                //Tape target
                if(aprilX.get() != ANGLE_ERROR_CODE && aprilY.get() != ANGLE_ERROR_CODE && 
                    aprilTimestamp.get() != ANGLE_ERROR_CODE &&
                    aprilTimestamp.get() < 200    
                ){
                    double visionDistanceX = Units.feetToMeters(aprilX.getAsDouble()) * APRIL_FLIP_X;
                    double visionDistanceY = Units.feetToMeters(aprilY.getAsDouble()) * APRIL_FLIP_Y;

                    Transform2d visionVector = new Transform2d(new Translation2d(visionDistanceX, visionDistanceY), new Rotation2d(0));

                    Pose2d robotRelative = APRIL_CAMERA_LOCATION.plus(visionVector);

                    Pose2d pose = SwerveSubsystem.getInstance().getPose();

                    Pose2d fieldRelativeTarget = pose.plus(new Transform2d(new Pose2d(), robotRelative));

                    calcAprilTargetX = linearAprilFilterX.calculate(fieldRelativeTarget.getX());
                    calcAprilTargetY = linearAprilFilterY.calculate(fieldRelativeTarget.getY());

                    aprilTimer.reset();
                    return new Translation2d(calcAprilTargetX, calcAprilTargetY);
                }
                else if(aprilTimer.hasElapsed(1)){
                    linearAprilFilterX.reset();
                    linearAprilFilterY.reset();
                }
                return null;
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

     private static Pose2d distAndYawToRobotPose(double visionX, double visionY, Rotation2d heading, Pose2d robotToCamera) {
        // Convert distance and yaw to XY
        Translation2d visionXY = new Translation2d(visionX, visionY);
        // Rotate XY from the camera's coordinate frame to the robot's coordinate frame
        Translation2d cameraToTargetRELATIVE = visionXY.rotateBy(robotToCamera.getRotation());
        // Add the XY from the robot center to camera and from the camera to the target
        Translation2d robotToTargetRELATIVE = robotToCamera.getTranslation().plus(cameraToTargetRELATIVE);
        // Rotate XY from the robot's coordinate frame to the field's coordinate frame and reverse the direction of the XY
        Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(
            heading
            );//.unaryMinus();

        // System.out.println(robotToTarget);
        // Add the XY from the field to the target and from the target to the robot center
        Translation2d fieldToRobot = SwerveSubsystem.getInstance().getPose().getTranslation().plus(robotToTarget);
        // Return the field to robot XY and set the gyro as the heading
        return new Pose2d(fieldToRobot, heading);
    } 
}
