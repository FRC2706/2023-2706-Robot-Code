// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2706.AdvantageUtil;
import frc.lib2706.LL3DApriltags;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.config.Config;

public class SwerveSubsystem extends SubsystemBase {
    private Field2d m_field = new Field2d();    


    private NetworkTable table = NetworkTableInstance.getDefault().getTable("DriveTrain");
    private DoublePublisher gyroEntry = table.getDoubleTopic("RawGyroYaw").publish();
    private DoublePublisher xEntry = table.getDoubleTopic("OdometryX").publish();
    private DoublePublisher yEntry = table.getDoubleTopic("OdometryY").publish();
    private DoubleEntry rotEntry = table.getDoubleTopic("OdometryRot").getEntry(0);

    private DoublePublisher pitchPub = table.getDoubleTopic("Pitch").publish();
    private DoublePublisher rollPub = table.getDoubleTopic("Roll").publish();

    private DoubleArrayPublisher pubEstimatedPose = table.getDoubleArrayTopic("EstimatedPose").publish();
    private DoubleArrayPublisher pubOdometry = table.getDoubleArrayTopic("OdometryPose").publish();
    
    // Instance for singleton class
    private static SwerveSubsystem instance;

    // Robot swerve modules
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_rearLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_rearRight;
    // The gyro sensor
    private final PigeonIMU m_pigeon; 

    // Odometry class for tracking robot pose
    private SwerveDriveOdometry m_odometry;
    private SwerveDrivePoseEstimator m_poseEstimator;

    private LL3DApriltags m_limelight;
    private boolean skipLimelightPoseChecks;

    /** Get instance of singleton class */
    public static SwerveSubsystem getInstance() {
        if (instance == null){
            SubsystemChecker.subsystemConstructed(SubsystemType.SwerveSubsystem);
            instance = new SwerveSubsystem();
        }

        return instance;
    }

    public void addVisionMeasurement(Pose2d pose, double timestamp){
        m_poseEstimator.addVisionMeasurement(pose, timestamp);
    }
    
    /** Creates a new DriveSubsystem. */
    private SwerveSubsystem() {
        m_frontLeft = new SwerveModule(Config.CANID.FRONT_LEFT_DRIVE, Config.Swerve.INVERTED_FRONT_LEFT_DRIVE, Config.CANID.FRONT_LEFT_STEERING, Config.Swerve.INVERTED_FRONT_LEFT_STEERING, Config.CANID.FRONT_LEFT_CANCODER, Config.Swerve.FL_ENCODER_OFFSET, "FL");

        m_rearLeft = new SwerveModule(Config.CANID.REAR_LEFT_DRIVE, Config.Swerve.INVERTED_REAR_LEFT_DRIVE, Config.CANID.REAR_LEFT_STEERING, Config.Swerve.INVERTED_REAR_LEFT_STEERING, Config.CANID.REAR_LEFT_CANCODER, Config.Swerve.RL_ENCODER_OFFSET, "RL");

        m_frontRight = new SwerveModule(Config.CANID.FRONT_RIGHT_DRIVE, Config.Swerve.INVERTED_FRONT_RIGHT_DRIVE, Config.CANID.FRONT_RIGHT_STEERING, Config.Swerve.INVERTED_FRONT_RIGHT_STEERING, Config.CANID.FRONT_RIGHT_CANCODER, Config.Swerve.FR_ENCODER_OFFSET, "FR");

        m_rearRight = new SwerveModule(Config.CANID.REAR_RIGHT_DRIVE, Config.Swerve.INVERTED_REAR_RIGHT_DRIVE, Config.CANID.REAR_RIGHT_STEERING, Config.Swerve.INVERTED_REAR_RIGHT_STEERING, Config.CANID.REAR_RIGHT_CANCODER, Config.Swerve.RR_ENCODER_OFFSET, "RR");
        m_pigeon = new PigeonIMU(Config.CANID.PIGEON);
        m_odometry = new SwerveDriveOdometry(Config.Swerve.kSwerveDriveKinematics, Rotation2d.fromDegrees(getGyro()), getPosition(), new Pose2d(0, 0, Rotation2d.fromDegrees(rotEntry.get())));
        SmartDashboard.putData("Field", m_field);
        m_poseEstimator = new SwerveDrivePoseEstimator(Config.Swerve.kSwerveDriveKinematics, Rotation2d.fromDegrees(getGyro()), getPosition(), new Pose2d());
    
        m_limelight = new LL3DApriltags("limelight");
    }

    @Override
    public void periodic() {
        double currentGyro = getGyro();

        m_frontLeft.updateNT();
        m_frontRight.updateNT();
        m_rearLeft.updateNT();
        m_rearRight.updateNT();


        // Update the odometry in the periodic block
        Pose2d odometryPose = m_odometry.update(
                Rotation2d.fromDegrees(currentGyro),
                getPosition()
        );
        
        Pose2d estimatedPose = m_poseEstimator.update(
                Rotation2d.fromDegrees(currentGyro),
                getPosition()
        );


        m_limelight.update(skipLimelightPoseChecks);

        pubEstimatedPose.accept(AdvantageUtil.deconstruct(estimatedPose));
        pubOdometry.accept(AdvantageUtil.deconstruct(odometryPose));
        
        gyroEntry.accept(currentGyro);
        xEntry.accept(getPose().getX());
        yEntry.accept(getPose().getY());
        rotEntry.accept(getPose().getRotation().getDegrees());

        m_field.setRobotPose(getPose());

        pitchPub.accept(getPitch());
        rollPub.accept(getRoll());
    }

    private SwerveModulePosition[] getPosition() {
        return new SwerveModulePosition[] {
            m_frontLeft.getModulePosition(),
            m_frontRight.getModulePosition(),
            m_rearLeft.getModulePosition(),
            m_rearRight.getModulePosition()};
    }

    public void setTrajectory(Trajectory traj){
        m_field.getObject("traj").setTrajectory(traj);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        System.out.println(pose.toString());
        m_odometry.resetPosition(Rotation2d.fromDegrees(getGyro()), getPosition(), pose);
        m_poseEstimator.resetPosition(Rotation2d.fromDegrees(getGyro()), getPosition(), pose);
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
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = Config.Swerve.kSwerveDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeading()));
        } else {
            swerveModuleStates = Config.Swerve.kSwerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Config.Swerve.kMaxAttainableWheelSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0], isOpenLoop);
        m_frontRight.setDesiredState(swerveModuleStates[1], isOpenLoop);
        m_rearLeft.setDesiredState(swerveModuleStates[2], isOpenLoop);
        m_rearRight.setDesiredState(swerveModuleStates[3],isOpenLoop);
    }

    public void setModuleStatesNoAntiJitter(SwerveModuleState[] desiredStates, boolean isOpenLoop){
        m_frontLeft.setDesiredState(desiredStates[0], isOpenLoop, false);
        m_frontRight.setDesiredState(desiredStates[1], isOpenLoop, false);
        m_rearLeft.setDesiredState(desiredStates[2], isOpenLoop, false);
        m_rearRight.setDesiredState(desiredStates[3], isOpenLoop, false);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Config.Swerve.kMaxAttainableWheelSpeed);
        m_frontLeft.setDesiredState(desiredStates[0], isOpenLoop);
        m_frontRight.setDesiredState(desiredStates[1], isOpenLoop);
        m_rearLeft.setDesiredState(desiredStates[2], isOpenLoop);
        m_rearRight.setDesiredState(desiredStates[3], isOpenLoop);
    }

    /**
     * Sets the swerve ModuleStates in auto. Defaults to closed loop.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStatesAuto(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
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
        return m_pigeon.getFusedHeading();
    }

    public SwerveModuleState[] getModuleStates()
    {
        return (new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()});

        
    }
    public ChassisSpeeds getSpeedsFieldRelative()
    {              
        return( ChassisSpeeds.fromFieldRelativeSpeeds(
            Config.Swerve.kSwerveDriveKinematics.toChassisSpeeds(getModuleStates()), getHeading()));
        
    }

    public ChassisSpeeds getSpeeds()
    {
        return Config.Swerve.kSwerveDriveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the heading of the robot.
     * 
     * Uses this method for heading. Odometry does an offset to ensure this has the correct origin.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public CommandBase getSyncOdometryCommand(){
        return Commands.runOnce(()-> resetOdometry(m_poseEstimator.getEstimatedPosition()));
    }

    public CommandBase getToggleChecksCommand(){
        return Commands.runOnce(()-> toggleSkipChecks());
    }

    public void toggleSkipChecks(){
        skipLimelightPoseChecks = !skipLimelightPoseChecks;
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

    /**
     * Checks if the Neo encoders are synced with their CanCoders
     * NOTE: This only works before the first enable since optimizing messes it up
     * 
     * @return Whether the encoders are synced or not
     */
    public boolean checkSteeringEncoders() {
        return m_frontLeft.areSteeringEncodersSynced() &&
                m_frontRight.areSteeringEncodersSynced() &&
                m_rearLeft.areSteeringEncodersSynced() &&
                m_rearRight.areSteeringEncodersSynced();
    }

    public void resetLastAngles() {
        m_frontLeft.resetLastAngle();
        m_frontRight.resetLastAngle();
        m_rearLeft.resetLastAngle();
        m_rearRight.resetLastAngle();
    }

    public double getRoll() {
        return(m_pigeon.getRoll());
    }

    public double getPitch() {
        return(m_pigeon.getPitch());
    }
}
