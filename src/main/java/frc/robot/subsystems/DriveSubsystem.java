// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class DriveSubsystem extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("DriveTrain");
    private NetworkTableEntry gyroEntry = table.getEntry("RawGyro");
    private NetworkTableEntry xEntry = table.getEntry("OdometryX");
    private NetworkTableEntry yEntry = table.getEntry("OdometryY");
    private NetworkTableEntry rotEntry = table.getEntry("OdometryRot");
    
    // Instance for singleton class
    private static DriveSubsystem instance;

    // Robot swerve modules
    private final SwerveModule m_frontLeft = new SwerveModule(Config.CANID_FRONT_LEFT_DRIVE, Config.INVERTED_FRONT_LEFT_DRIVE, Config.CANID_FRONT_LEFT_STEERING, Config.INVERTED_FRONT_LEFT_STEERING, Config.CANID_FRONT_LEFT_CANCODER, Config.FL_ENCODER_OFFSET, "FL");

    private final SwerveModule m_rearLeft = new SwerveModule(Config.CANID_REAR_LEFT_DRIVE, Config.INVERTED_REAR_LEFT_DRIVE, Config.CANID_REAR_LEFT_STEERING, Config.INVERTED_REAR_LEFT_STEERING, Config.CANID_REAR_LEFT_CANCODER, Config.RL_ENCODER_OFFSET, "RL");

    private final SwerveModule m_frontRight = new SwerveModule(Config.CANID_FRONT_RIGHT_DRIVE, Config.INVERTED_FRONT_RIGHT_DRIVE, Config.CANID_FRONT_RIGHT_STEERING, Config.INVERTED_FRONT_RIGHT_STEERING, Config.CANID_FRONT_RIGHT_CANCODER, Config.FR_ENCODER_OFFSET, "FR");

    private final SwerveModule m_rearRight = new SwerveModule(Config.CANID_REAR_RIGHT_DRIVE, Config.INVERTED_REAR_RIGHT_DRIVE, Config.CANID_REAR_RIGHT_STEERING, Config.INVERTED_REAR_RIGHT_STEERING, Config.CANID_REAR_RIGHT_CANCODER, Config.RR_ENCODER_OFFSET, "RR");

    // The gyro sensor
    private final PigeonIMU gyro; 

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry;

    /** Get instance of singleton class */
    public static DriveSubsystem getInstance() {
        if (instance == null)
            instance = new DriveSubsystem();
        return instance;
    }
    
    /** Creates a new DriveSubsystem. */
    private DriveSubsystem() {
        gyro = new PigeonIMU(Config.CAN_PIGEON);
        m_odometry = new SwerveDriveOdometry(Config.kSwerveDriveKinematics, Rotation2d.fromDegrees(getGyro()));
    }

    @Override
    public void periodic() {
        double currentGyro = getGyro();

        // Update the odometry in the periodic block
        m_odometry.update(
                Rotation2d.fromDegrees(currentGyro),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
        
        gyroEntry.setDouble(currentGyro);
        xEntry.setDouble(getPose().getX());
        yEntry.setDouble(getPose().getY());
        rotEntry.setDouble(getPose().getRotation().getDegrees());
        
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getGyro()));
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = Config.kSwerveDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()));
        } else {
            swerveModuleStates = Config.kSwerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Config.kMaxAttainableWheelSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Config.kMaxAttainableWheelSpeed);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Returns the heading of the robot.
     * 
     * This is private because only the odoemtry get's the raw gyro value. 
     * Everything else get's the gyro value from the odometry since it does an offset.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    private double getGyro() {
        return gyro.getYaw();
    }

    /**
     * Returns the heading of the robot.
     * 
     * Uses this method for heading. Odometry does an offset to ensure this has the correct origin.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return m_odometry.getPoseMeters().getRotation();
    }

    /**
     * Standard stop motors method for every subsystem.
     */
    public void stopMotors() {
        m_frontLeft.stopMotors();
        m_rearLeft.stopMotors();
        m_frontRight.stopMotors();
        m_rearRight.stopMotors();
    }

    public void updateModulesPID(){
        m_frontLeft.updatePIDValues();
        m_frontRight.updatePIDValues();
        m_rearLeft.updatePIDValues();
        m_rearRight.updatePIDValues();
    }

    public void resetEncodersFromCanCoder() {
        m_frontLeft.updateSteeringFromCanCoder();
        m_frontRight.updateSteeringFromCanCoder();
        m_rearLeft.updateSteeringFromCanCoder();
        m_rearRight.updateSteeringFromCanCoder();
    }

}
