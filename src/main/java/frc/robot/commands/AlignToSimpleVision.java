// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToSimpleVision extends CommandBase {
    Pose2d ROBOT_TO_CAMERA = new Pose2d();
    Pose2d desiredPose;

    /** Creates a new AlignToSimpleVision. */
    public AlignToSimpleVision() {
        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SwerveSubsystem.getInstance().resetOdometryPID();
        desiredPose = SwerveSubsystem.getInstance().getOdometryPose();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        boolean hasTarget = Math.random() > 0.5;

        if (hasTarget) {
            double timestampSeconds = 0;
            Rotation2d yaw = Rotation2d.fromDegrees(0);
            double distance = 0;
    
            Optional<Pose2d> opt = SwerveSubsystem.getInstance().getPoseAtTimestamp(timestampSeconds);
            if (opt.isEmpty()) {
                return;
            }
            Pose2d poseAtTimestamp = opt.get();
            
            Translation2d robotToTarget = calcRobotToTarget(distance, yaw, poseAtTimestamp.getRotation(), ROBOT_TO_CAMERA);
            Translation2d targetTranslation = poseAtTimestamp.getTranslation().plus(robotToTarget);
            Rotation2d targetRotation = poseAtTimestamp.getRotation().plus(yaw);
            desiredPose = new Pose2d(targetTranslation, targetRotation);
        }
        
        SwerveSubsystem.getInstance().driveToPose(desiredPose, true);
        

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().stopMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return SwerveSubsystem.getInstance().hasReachedOdometryPID();
    }

    /**
     * Calculate the a translation2d in the field coordinate frame from the robot to the target.
     *
     * @param visionDistance Distance from the camera to the target in the camera's
     *                       coordinate frame
     * @param visionYaw      Yaw of the camera to the target in the camera's
     *                       coordinate frame
     * @param heading        Heading of the robot measured by the gyro
     * @param fieldToTarget  Pose3d of the target in the field's coordinate frame.
     *                       Use AprilTagFieldLayout.java for this.
     * @param robotToCamera  Pose2d offset of the camera in the robot's coordinate
     *                       frame
     *
     * @return Translation in the field's coordinate frame between the robot and the target.
     */

    private static Translation2d calcRobotToTarget(double visionDistance, Rotation2d visionYaw, Rotation2d heading,
            Pose2d robotToCamera) {
        // Convert distance and yaw to XY
        Translation2d visionXY = new Translation2d(visionDistance, visionYaw);

        // Rotate XY from the camera's coordinate frame to the robot's coordinate frame
        Translation2d cameraToTargetRELATIVE = visionXY.rotateBy(robotToCamera.getRotation());

        // Add the XY from the robot center to camera and from the camera to the target
        Translation2d robotToTargetRELATIVE = robotToCamera.getTranslation().plus(cameraToTargetRELATIVE);

        // Rotate XY from the robot's coordinate frame to the field's coordinate frame
        Translation2d robotToTarget = robotToTargetRELATIVE.rotateBy(heading);

        return robotToTarget;
    }
}
