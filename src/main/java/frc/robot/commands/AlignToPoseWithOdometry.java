// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToPoseWithOdometry extends CommandBase {
    final double VEL_TOLERANCE = 0.1;
    final double ANGULAR_VEL_TOLERANCE = Math.toRadians(10);

    final double TIGHT_POS_TOLERANCE = 0.01;
    final double TIGHT_ROT_TOLERANCE = Math.toRadians(3);
    final double LOOSE_POS_TOLERANCE = 0.3;
    final double LOOSE_ROT_TOLERANCE = Math.toRadians(10);

    ProfiledPIDController pidX = new ProfiledPIDController(4, 0.0, 0.2,
            new TrapezoidProfile.Constraints(2, 2));
    ProfiledPIDController pidY = new ProfiledPIDController(4, 0.0, 0.2,
            new TrapezoidProfile.Constraints(2, 2));
    ProfiledPIDController pidRot = new ProfiledPIDController(4.0, 0, 0.4,
            new TrapezoidProfile.Constraints(4 * Math.PI, 8 * Math.PI));

    private Pose2d m_targetPose;
    private boolean m_isWaypoint;
    private boolean m_neverEnd;

    final double POS_TOLERANCE;
    final double ROT_TOLERANCE;

    public AlignToPoseWithOdometry(Pose2d targetPose, boolean isWaypoint, boolean neverEnd) {
        this(targetPose, isWaypoint, neverEnd, 2, 2);
    }

    /** Creates a new AlignWithNode. */
    public AlignToPoseWithOdometry(Pose2d targetPose, boolean isWaypoint, boolean neverEnd, double vel, double accel) {
        m_targetPose = targetPose;
        m_isWaypoint = isWaypoint;
        m_neverEnd = neverEnd;
        pidX.setConstraints(new Constraints(vel, accel));
        pidY.setConstraints(new Constraints(vel, accel));
        pidRot.enableContinuousInput(-Math.PI, Math.PI);

        if (isWaypoint) {
            POS_TOLERANCE = LOOSE_POS_TOLERANCE;
            ROT_TOLERANCE = LOOSE_ROT_TOLERANCE;
        } else {
            POS_TOLERANCE = TIGHT_POS_TOLERANCE;
            ROT_TOLERANCE = TIGHT_ROT_TOLERANCE;
        }

        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ChassisSpeeds speeds = SwerveSubsystem.getInstance().getSpeedsFieldRelative();
        Pose2d pose = SwerveSubsystem.getInstance().getPose();
        pidX.reset(pose.getX(), speeds.vxMetersPerSecond);
        pidY.reset(pose.getY(), speeds.vyMetersPerSecond);
        pidRot.reset(pose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Pose2d pose = SwerveSubsystem.getInstance().getPose();
        double xSpeed = pidX.calculate(pose.getX(), m_targetPose.getX());
        double ySpeed = pidY.calculate(pose.getY(), m_targetPose.getY());
        double rot = pidRot.calculate(pose.getRotation().getRadians(),
                m_targetPose.getRotation().getRadians());

        if (Math.abs(pose.getX() - m_targetPose.getX()) < TIGHT_POS_TOLERANCE) {
            xSpeed = 0;
        }
        if (Math.abs(pose.getY() - m_targetPose.getY()) < TIGHT_POS_TOLERANCE) {
            ySpeed = 0;
        }
        if (Math.abs(pose.getRotation().getRadians() -
                    m_targetPose.getRotation().getRadians()) < TIGHT_ROT_TOLERANCE) {
            rot = 0;
        }
        
        
        SwerveSubsystem.getInstance().drive(
                xSpeed,
                ySpeed,
                rot,
                true,
                false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!m_isWaypoint) {
            SwerveSubsystem.getInstance().stopMotors();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Check if command should never return true. (Something else has to cancel it)
        if (m_neverEnd) {
            return false;
        }

        ChassisSpeeds speeds = SwerveSubsystem.getInstance().getSpeedsFieldRelative();
        Pose2d pose = SwerveSubsystem.getInstance().getPose();

        boolean atSetpoint = (Math.abs(pose.getX() - m_targetPose.getX()) < POS_TOLERANCE) &&
                (Math.abs(pose.getY() - m_targetPose.getY()) < POS_TOLERANCE) &&
                (Math.abs(pose.getRotation().getRadians() - m_targetPose.getRotation().getRadians()) < ROT_TOLERANCE);

        // If it's a waypoint command and we're within the tolerance, stop running.
        if (m_isWaypoint && atSetpoint) {
            return true;
        }

        boolean notMoving = (Math.abs(speeds.vxMetersPerSecond) < VEL_TOLERANCE) &&
                (Math.abs(speeds.vyMetersPerSecond) < VEL_TOLERANCE) &&
                (Math.abs(speeds.omegaRadiansPerSecond) < ANGULAR_VEL_TOLERANCE);

        // Not a waypoint, so the robot must be still and at the setpoint.
        return notMoving && atSetpoint;
    }
}
