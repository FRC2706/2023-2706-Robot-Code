// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.config.Config;

public class DiffNeoSubsystem extends SubsystemBase {
    private static DiffNeoSubsystem instance;

    public static DiffNeoSubsystem getInstance() {
        if (instance == null) {
            if (Config.DIFF.ISNEOS == false) { 
                DriverStation.reportError(
                    String.format("DiffNeoSubsystem.getInstance() was called even though Config.DIFF_ISNEOS is false. RobotID: %d", Config.getRobotId()), 
                    true);
            }
            SubsystemChecker.subsystemConstructed(SubsystemType.DiffNeoSubsystem);
            instance = new DiffNeoSubsystem();
        }
        return instance;
    }

    // SparkMax's
    private CANSparkMax leftLeader, rightLeader, leftFollower, rightFollower;

    // Gyro
    private PigeonIMU pigeon;

    // DifferentialDrive for teleop control
    private DifferentialDrive differentialDrive; 
    
    // Odometry to track the robot's location on the field
    private DifferentialDriveOdometry odometry;

    // NetworkTable Values
    private NetworkTableEntry xOdometry, yOdometry, rotOdometry;

    /** Creates a new DifferentialNeoSubsystem. */
    private DiffNeoSubsystem() {
        /*
         * Construct SparkMaxs and pass settings 
         */
        leftLeader = new CANSparkMax(Config.CANID.DIFF_LEADER_LEFT, MotorType.kBrushless);
        rightLeader = new CANSparkMax(Config.CANID.DIFF_LEADER_RIGHT, MotorType.kBrushless);
        leftFollower = new CANSparkMax(Config.CANID.DIFF_FOLLOWER_LEFT, MotorType.kBrushless);
        rightFollower = new CANSparkMax(Config.CANID.DIFF_FOLLOWER_RIGHT, MotorType.kBrushless);

        // Restore the SparkMax's to a know group of settings
        leftLeader.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        // Make the followers copy the leaders
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // Invert the motor direction
        leftLeader.setInverted(Config.DIFF.LEADER_LEFT_INVERTED);
        rightLeader.setInverted(Config.DIFF.LEADER_RIGHT_INVERTED);
        leftFollower.setInverted(Config.DIFF.FOLLOWER_LEFT_INVERTED);
        rightFollower.setInverted(Config.DIFF.FOLLOWER_RIGHT_INVERTED);

        // Set brake or coast mode. When the motor is doing nothing, brake mode applies a brake
        leftLeader.setIdleMode(Config.DIFF.TELEOP_IDLEMODE);
        rightLeader.setIdleMode(Config.DIFF.TELEOP_IDLEMODE);
        leftFollower.setIdleMode(Config.DIFF.TELEOP_IDLEMODE);
        rightFollower.setIdleMode(Config.DIFF.TELEOP_IDLEMODE);

        // Position conversion factor to change the units
        leftLeader.getEncoder().setPositionConversionFactor(Config.DIFF.ROTATIONS_TO_METERS);
        rightLeader.getEncoder().setPositionConversionFactor(Config.DIFF.ROTATIONS_TO_METERS);
        leftFollower.getEncoder().setPositionConversionFactor(Config.DIFF.ROTATIONS_TO_METERS);
        rightFollower.getEncoder().setPositionConversionFactor(Config.DIFF.ROTATIONS_TO_METERS);

        // Velocity conversion factor to change the units
        leftLeader.getEncoder().setVelocityConversionFactor(Config.DIFF.RPM_TO_METERS_PER_SECOND);
        rightLeader.getEncoder().setVelocityConversionFactor(Config.DIFF.RPM_TO_METERS_PER_SECOND);
        leftFollower.getEncoder().setVelocityConversionFactor(Config.DIFF.RPM_TO_METERS_PER_SECOND);
        rightFollower.getEncoder().setVelocityConversionFactor(Config.DIFF.RPM_TO_METERS_PER_SECOND);

        // Prevents the motors from drawing more voltage than the specified amount
        leftLeader.enableVoltageCompensation(Config.DIFF.DRIVESUBSYSTEM_VOLTAGECOMP);
        rightLeader.enableVoltageCompensation(Config.DIFF.DRIVESUBSYSTEM_VOLTAGECOMP);
        leftFollower.enableVoltageCompensation(Config.DIFF.DRIVESUBSYSTEM_VOLTAGECOMP);
        rightFollower.enableVoltageCompensation(Config.DIFF.DRIVESUBSYSTEM_VOLTAGECOMP);

        // Prevent the motors from drawing more current than the specified amount
        leftLeader.setSmartCurrentLimit(Config.DIFF.NEO_DRIVER_CURRENTLIMIT);
        rightLeader.setSmartCurrentLimit(Config.DIFF.NEO_DRIVER_CURRENTLIMIT);
        leftFollower.setSmartCurrentLimit(Config.DIFF.NEO_DRIVER_CURRENTLIMIT);
        rightFollower.setSmartCurrentLimit(Config.DIFF.NEO_DRIVER_CURRENTLIMIT);

        // Set the P constant for the PID controller
        leftLeader.getPIDController().setP(Config.DIFF.kRamsetePGain, 1);
        rightLeader.getPIDController().setP(Config.DIFF.kRamsetePGain, 1);
        leftFollower.getPIDController().setP(Config.DIFF.kRamsetePGain, 1);
        rightFollower.getPIDController().setP(Config.DIFF.kRamsetePGain, 1);

        /*
         * Construct other objects
         */
        pigeon = new PigeonIMU(Config.CANID.PIGEON);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getCurrentAngle()), getLeftPosition(), getRightPosition(), getPose()); 

        differentialDrive = new DifferentialDrive(leftLeader, rightLeader);

        var table = NetworkTableInstance.getDefault().getTable("DrivetrainOdometry");
        xOdometry = table.getEntry("xOdometry");
        yOdometry = table.getEntry("yOdometry");
        rotOdometry = table.getEntry("rotOdometry"); 
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        /** 
         * Give heading (from gyro) and encoder data in meters to odometry to calculate a new robot pose.
         */
        Pose2d newPose = odometry.update(Rotation2d.fromDegrees(getCurrentAngle()), getLeftPosition(), getRightPosition());
    
        xOdometry.setDouble(newPose.getX());
        yOdometry.setDouble(newPose.getY());
        rotOdometry.setDouble(newPose.getRotation().getDegrees());
    }

    /**
     * Arcade drive method for a differential drive robot.
     *
     * @param forward The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param steering The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is
     *     positive.
     */
    public void arcadeDrive(double forward, double steering) {
        differentialDrive.arcadeDrive(forward, steering);
    }

    /**
     *  Stop the motors from moving until told to move again.
     */
    public void stopMotors() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();
        leftFollower.stopMotor();
        rightFollower.stopMotor();
    }

    /**
     * Get the encoder data in meters
     */
    private double getLeftPosition() {
        return leftLeader.getEncoder().getPosition();
    }

    /**
     * Get the encoder data in meters
     */
    private double getRightPosition() {
        return rightLeader.getEncoder().getPosition();
    }

    /**
     * Get the fused heading from the pigeon
     * 
     * @return Heading of the robot in degrees
     */
    private double getCurrentAngle() {
        return pigeon.getFusedHeading();
    }

    /**
     * Returns the a Pose2d of the current robot location
     * 
     * Odometry calculates a new pose every robot cycle and stores
     * the value so this method is only reading the stored value.
     * This means it already does only 1 hardware read every cycle instead of 
     * many things calling hardware redundantly.
     * 
     * @param Pose2d the current pose of the robot
     */
    public Pose2d getPose() { 
        return odometry.getPoseMeters();
    }

    /**
     * This method will return the heading from odometry
     * 
     * Odometry keeps track of the gyro heading and in relation to
     * the value it was reset to using an offset so it's important to ask
     * the odometry for the rotation instead of directly from the gyro.
     * 
     * @param Rotation2d The heading. Rotation2d has a .getDegrees() & a .getRadians() method.
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Set the odometry to a given pose.
     * 
     * Necessary to do at the beginning of a match.
     * 
     * Very important to set the encoders to 0 when resetting odometry.
     * 
     * @param newPose New pose to set odometry to.
     */
    public void resetPose(Pose2d newPose) {
        leftLeader.getEncoder().setPosition(0);
        rightLeader.getEncoder().setPosition(0);
        odometry.resetPosition(Rotation2d.fromDegrees(getCurrentAngle()), 0, 0, newPose);
    }

    /**
     * Get the velocity of the left side of the drivetrain
     * @return meters/second
     */
    public double getLeftVelocity() {
        return leftLeader.getEncoder().getVelocity();
    }

    /**
     * Get the velocity of the right side of the drivetrain
     * @return meters/second
     */
    public double getRightVelocity() {
        return rightLeader.getEncoder().getVelocity();
    }

    /**
     * Set the left and right volts of the drivetrain while running a small P loop to make sure 
     * the correct velocity is achieved.
     * 
     * This method of using SimpleMotorFeedforward and frc-characterization is copied from
     * RamseteCommand {@link RamseteCommand}. However, instead of running a PIDController in RamseteCommand, 
     * the PID loop is run on the SparkMax using ControlType.kVelocity. Only a P gain is required
     * since it only has to compensate for any error in SimpleMotorFeedforward.
     * 
     * See writeup on frc-characterization for more info on that equation and how to characterize.
     * 
     * @param leftVelocity Meters per second for the left side.
     * @param rightVelocity Meters per second for the right side.
     */
    public void setRamsete(double leftVolts, double rightVolts, double leftVel, double rightVel) {
        leftLeader.getPIDController().setReference(leftVel, ControlType.kVelocity, 1, leftVolts);
        rightLeader.getPIDController().setReference(rightVel, ControlType.kVelocity, 1, rightVolts);

        // Make sure motor safety knows the motors are being used
        differentialDrive.feed();

    }

    public void setIdleMode(IdleMode mode) {

    } 
}
