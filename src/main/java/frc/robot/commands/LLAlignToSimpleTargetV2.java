// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSub;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.libLimelight.LimelightHelpers;
import frc.robot.config.Config;
import frc.robot.subsystems.SwerveSubsystem;

public class LLAlignToSimpleTargetV2 extends CommandBase {
    private final double TIME_WITH_NO_DATA = 0.4;
    private final double KEEP_AWAY_DISTANCE_METERS = 0.7;
    private final double KEEP_AWAY_TOLERANCE_RAD = Math.toRadians(3);

    private final double DESIRED_Y_DISTANCE_METERS = 0; // MUST BE ZERO FOR CURRENT DESIGN TO WORK
    private final double DESIRED_ROT_TO_TARGET_RAD = 0; // MUST BE ZERO FOR CURRENT DESIGN TO WORK
    
    private final double CLAMP_SPEED_TOLERANCE = 0.2;
    private final double CLAMP_SPEED_VALUE = 0.03;

    private final double CLAMP_ROT_SPEED_TOLERANCE = Math.toRadians(3);
    private final double CLAMP_ROT_SPEED_VALUE = Math.toRadians(10);

    ProfiledPIDController pidX = new ProfiledPIDController(2, 0.0, 0,
        new Constraints(2, 0.4));
        // new Constraints(1, 0.5));
    ProfiledPIDController pidY = new ProfiledPIDController(2, 0.0, 0,
        new Constraints(2, 0.4));
        // new Constraints(1, 0.5));
    ProfiledPIDController pidRot = new ProfiledPIDController(2, 0, 0,
        // new Constraints(3 * Math.PI, 3 * Math.PI));
        new Constraints(4 * Math.PI, 8 * Math.PI));
    // PIDController pidRot = new PIDController(0.2, 0, 0);

    private final String m_llName;
    private final double m_desiredDistanceMeters;
    private final Rotation2d m_targetRotation;
    private final Rotation2d m_limelightAngle;
    private final Supplier<Double> m_distanceToTarget;
    private final Timer m_timer;

    private double xSpeed, ySpeed, rotSpeed;
    private Rotation2d angleAtTarget;
    private boolean initializePid = false;

    private DoublePublisher pubXDistance, pubYDistance, pubDesiredX, pubAngleAtTarget, pubDistance;

    /** Creates a new LLAlign. */
    public LLAlignToSimpleTargetV2(String llName, double desiredDistanceMeters, Rotation2d desiredLimelightHeading, Rotation2d limelightAngle, Supplier<Double> distanceToTarget) {
        m_llName = llName;
        m_desiredDistanceMeters = desiredDistanceMeters;
        m_targetRotation = desiredLimelightHeading;
        m_limelightAngle = limelightAngle;
        m_distanceToTarget = distanceToTarget;
        m_timer = new Timer();

        pidRot.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(SwerveSubsystem.getInstance());

        NetworkTable table = NetworkTableInstance.getDefault().getTable("LLAlignToSimpleTarget");
        pubXDistance = table.getDoubleTopic("xDistance").publish(PubSubOption.periodic(0.02));
        pubYDistance = table.getDoubleTopic("yDistance").publish(PubSubOption.periodic(0.02));
        pubDesiredX = table.getDoubleTopic("desiredX").publish();
        pubAngleAtTarget = table.getDoubleTopic("angleAtTarget").publish(PubSubOption.periodic(0.02));
        pubDistance = table.getDoubleTopic("distance").publish(PubSubOption.periodic(0.02));
    }

    public static Supplier<Double> createApriltagDistanceSupplier(String llName) {
        return () -> {
            return (1.43 / (LimelightHelpers.getTLONG(llName) / 52));
        };
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        xSpeed = 0;
        ySpeed = 0;
        rotSpeed = 0;
        angleAtTarget = new Rotation2d(0);
        initializePid = true;

        m_timer.restart();

        
        // Pose2d pose = SwerveSubsystem.getInstance().getPose();
        // pidX.reset(pose.getX(), speeds.vxMetersPerSecond);
        // pidY.reset(pose.getY(), speeds.vyMetersPerSecond);
        // pidRot.reset(pose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);

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

            // Distance in meters to the target
            double llDistanceToTargetMeters = m_distanceToTarget.get();
            pubDistance.accept(llDistanceToTargetMeters);

            // Heading of the limelight in field coordinate frame
            Rotation2d llHeading = SwerveSubsystem.getInstance().getHeading().plus(m_limelightAngle);

            // Angle between a line drawn straight out of the TARGET and a line draw between the limelight and the target
            angleAtTarget = llHeading.minus(m_targetRotation);
            pubAngleAtTarget.accept(angleAtTarget.getDegrees());

            Translation2d translation = new Translation2d(llDistanceToTargetMeters, angleAtTarget);
            double currentX = translation.getX();
            double currentY = translation.getY();
            pubXDistance.accept(currentX);
            pubYDistance.accept(currentY);

            // Move in the Y until the limelight is directly facing the target
            // double distanceParallelToTarget = llDistanceToTargetMeters * Math.sin(angleAtTarget.getRadians());
            // pubYDistance.accept(distanceParallelToTarget);

            if (initializePid) {
                ChassisSpeeds speeds = SwerveSubsystem.getInstance().getSpeedsFieldRelative();
                pidX.reset(currentX, speeds.vxMetersPerSecond);
                pidY.reset(currentY, speeds.vyMetersPerSecond);
                pidRot.reset(angleAtLimelightRad, speeds.omegaRadiansPerSecond);
                initializePid = false;
            }

            ySpeed = pidY.calculate(currentY, DESIRED_Y_DISTANCE_METERS);
            if (Math.abs(currentY-DESIRED_Y_DISTANCE_METERS) < CLAMP_SPEED_TOLERANCE) {
                ySpeed = MathUtil.clamp(ySpeed, -CLAMP_SPEED_VALUE, CLAMP_SPEED_VALUE);
            }



            // Rotate the robot to face the target
            rotSpeed = pidRot.calculate(
                angleAtLimelightRad, 
                DESIRED_ROT_TO_TARGET_RAD);

            if (Math.abs(angleAtLimelightRad) < CLAMP_ROT_SPEED_TOLERANCE) {
                rotSpeed = MathUtil.clamp(rotSpeed, -CLAMP_ROT_SPEED_VALUE, CLAMP_ROT_SPEED_VALUE);
            }

            double desiredX;
            // Keep away from the target if true
            if (Math.abs(angleAtTarget.getRadians()) > KEEP_AWAY_TOLERANCE_RAD) {
                desiredX = KEEP_AWAY_DISTANCE_METERS / Math.cos(angleAtTarget.getRadians());

            // Directly facing target so approach the target
            } else {
                desiredX = m_desiredDistanceMeters;
            }
            pubDesiredX.accept(desiredX);

            xSpeed = pidX.calculate(currentX, desiredX);
            if (Math.abs(currentX - desiredX) < CLAMP_SPEED_TOLERANCE) {
                xSpeed = MathUtil.clamp(xSpeed, -CLAMP_SPEED_VALUE, CLAMP_SPEED_VALUE);
            }
        }
        
        // Drive using an X and Y with respect to the target.
        // driveFromTarget(-xSpeed, -ySpeed, rotSpeed, angleAtTarget);

        SwerveSubsystem.getInstance().drive(
            xSpeed, 
            ySpeed, 
            rotSpeed, 
            true, 
            false);
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

    /**
     * Takes target coordinate frame speeds to the drive the robot.
     * Aka rotates the xSpeed and ySpeed into robot coordinate frame.
     * 
     * @param xSpeed Meters per second
     * @param ySpeed Meters per second
     * @param rotSpeed Radians per second
     * @param angleAtTarget Angle between a line drawn straight out of 
     *    the TARGET and a line draw between the limelight and the target
     */
    private void driveFromTarget(double xSpeed, double ySpeed, double rotSpeed, Rotation2d angleAtTarget) {
        SwerveModuleState[] swerveModuleStates = Config.Swerve.kSwerveDriveKinematics.toSwerveModuleStates(
            // Hack to convert speeds between coordinate frames
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, 
                ySpeed, 
                rotSpeed, 
                m_limelightAngle.unaryMinus().minus(angleAtTarget)));
        
        SwerveSubsystem.getInstance().setModuleStates(swerveModuleStates, false);
    }
}
