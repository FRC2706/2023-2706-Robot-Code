// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignToNearestConeNode extends CommandBase {
    private final double SCORING_X = 1.8;
    private final Rotation2d SCORING_ANGLE = Rotation2d.fromDegrees(180);

    private AlignToPoseWithOdometry alignCommand;

    private double m_vel, m_accel;
    private final boolean m_useDefaultSpeeds;

    public AlignToNearestConeNode() {
        m_useDefaultSpeeds = true;
    }

    /** Creates a new AlignToNearestConeNode. */
    public AlignToNearestConeNode(double vel, double accel) {
        m_vel = vel;
        m_accel = accel;
        m_useDefaultSpeeds = false;

        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Alliance alliance = DriverStation.getAlliance();
        double robotY = SwerveSubsystem.getInstance().getPose().getY();
        
        double[] yOfNodes;
        if (alliance == Alliance.Red) {
            yOfNodes = Config.CONE_Y_COORDINATES_RED;
        } else {
            yOfNodes = Config.CONE_Y_COORDINATES_BLUE;
        }

        double distanceToRobotY = 100;
        double nodeY = robotY;
        for (int i = 0; i < yOfNodes.length; i++) {
            double distance = Math.abs(yOfNodes[i] - robotY);
            if (distance < distanceToRobotY) {
                distanceToRobotY = distance;
                nodeY = yOfNodes[i];
            }
        }

        if (distanceToRobotY >= 99) {
            this.cancel();
        }

        if (m_useDefaultSpeeds) {
            alignCommand = new AlignToPoseWithOdometry(
                new Pose2d(SCORING_X, nodeY, SCORING_ANGLE), 
                false, 
                false);
        } else {
            alignCommand = new AlignToPoseWithOdometry(
                new Pose2d(SCORING_X, nodeY, SCORING_ANGLE), 
                false, 
                false, 
                m_vel, 
                m_accel);
        }

        alignCommand.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        alignCommand.execute();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        alignCommand.end(interrupted);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return alignCommand.isFinished();
    }
}
