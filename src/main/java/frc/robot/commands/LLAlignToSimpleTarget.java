// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.libLimelight.LimelightHelpers;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class LLAprilTagAlign extends CommandBase {
    private final double TIME_WITH_NO_DATA = 0.4;
    private final double KEEP_AWAY_DISTANCE_METERS = 1.2;
    private final double KEEP_AWAY_TOLERANCE_RAD = Math.toRadians(5);

    private final double DESIRED_Y_DISTANCE_METERS = 0; // MUST BE ZERO FOR CURRENT DESIGN TO WORK
    private final double DESIRED_ROT_TO_TARGET_RAD = 0; // MUST BE ZERO FOR CURRENT DESIGN TO WORK
    

    ProfiledPIDController pidX = new ProfiledPIDController(4, 0.0, 0.2,
        new Constraints(2, 2));
    ProfiledPIDController pidY = new ProfiledPIDController(4, 0.0, 0.2,
        new Constraints(2, 2));
    ProfiledPIDController pidRot = new ProfiledPIDController(5.0, 0, 0.4,
        new Constraints(4 * Math.PI, 8 * Math.PI));

    private final String m_llName;
    private final double m_desiredDistanceMeters;
    private final Rotation2d m_targetRotation;
    private final Rotation2d m_limelightAngle;
    private final Timer m_timer;

    private double xSpeed, ySpeed, rotSpeed;

    /** Creates a new LLAlign. */
    public LLAprilTagAlign(String llName, double desiredDistanceMeters, Rotation2d targetRotation, Rotation2d limelightAngle) {
        m_llName = llName;
        m_desiredDistanceMeters = desiredDistanceMeters;
        m_targetRotation = targetRotation;
        m_limelightAngle = limelightAngle;
        m_timer = new Timer();

        addRequirements(SwerveSubsystem.getInstance());
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xSpeed = 0;
        ySpeed = 0;
        rotSpeed = 0;

        m_timer.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Only update speeds if a target is in view
        if (LimelightHelpers.getTV(m_llName)) {
            // Reset timer so it knows we have data
            m_timer.reset();

            // Angle between a line drawn straight out of the LIMELIGHT and a line draw between the limelight and the target
            double angleAtLimelightRad = Math.toRadians(LimelightHelpers.getTX(m_llName));

            // Rotate the robot to face the target
            rotSpeed = pidRot.calculate(angleAtLimelightRad, DESIRED_ROT_TO_TARGET_RAD);

            // Heading of the limelight in field coordinate frame
            Rotation2d llHeading = SwerveSubsystem.getInstance().getHeading().plus(m_limelightAngle);

            // Angle between a line drawn straight out of the TARGET and a line draw between the limelight and the target
            Rotation2d angleAtTarget = llHeading.minus(m_targetRotation);

            // Distance in meters to the target
            double llDistanceToTargetMeters = (1.43 / (LimelightHelpers.getTLONG(m_llName) / 52));

            // Move in the Y until the limelight is directly facing the target
            double distanceParallelToTarget = llDistanceToTargetMeters * Math.sin(angleAtTarget.getRadians());
            ySpeed = pidY.calculate(distanceParallelToTarget, DESIRED_Y_DISTANCE_METERS);
            
            // Keep away from the target if true
            if (Math.abs(angleAtTarget.getRadians()) > KEEP_AWAY_TOLERANCE_RAD) {
                double temporaryDesiredDistance = KEEP_AWAY_DISTANCE_METERS / Math.cos(angleAtTarget.getRadians());
                xSpeed = pidX.calculate(llDistanceToTargetMeters, temporaryDesiredDistance);

            // Directly facing target so approach the target
            } else {
                xSpeed = pidX.calculate(llDistanceToTargetMeters, m_desiredDistanceMeters);
            }
        }
        
        // Drive from the limelights coordinate frame to simplify testing on Poseidon. 
        driveFromLimelight(xSpeed, ySpeed, rotSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SwerveSubsystem.getInstance().stopMotors();
        }

        m_timer.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(TIME_WITH_NO_DATA);
    }

    /**
     * Takes limelight coordinate frame speeds to the drive the robot.
     * Aka rotates the xSpeed and ySpeed into robot coordinate frame.
     * 
     * @param xSpeed Meters per second
     * @param ySpeed Meters per second
     * @param rotSpeed Radians per second
     */
    private void driveFromLimelight(double xSpeed, double ySpeed, double rotSpeed) {
        SwerveModuleState[] swerveModuleStates = Config.Swerve.kSwerveDriveKinematics.toSwerveModuleStates(
            // Hack to convert speeds between coordinate frames
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, 
                ySpeed, 
                rotSpeed, 
                m_limelightAngle.unaryMinus()));
        
        SwerveSubsystem.getInstance().setModuleStates(swerveModuleStates, false);
    }
}
