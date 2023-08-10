// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2706.AdvantageUtil;
import frc.lib2706.CTREUnits;
import frc.lib2706.DifferentialDrivePoseEstimatorExposed;
import frc.lib2706.LL3DApriltags;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.config.Config;

public class DiffTalonSubsystem extends SubsystemBase {
    /**
     * Instance Variables
     */
    private WPI_TalonSRX leftLeader, rightLeader;    
    private BaseMotorController leftFollower, rightFollower;

    private DifferentialDrive diffDrive;
    
    private PigeonIMU pigeon;

    private DifferentialDrivePoseEstimatorExposed poseEstimator;
    private boolean disableVisionFeedback = false;

    private LL3DApriltags limelight;

    private DoublePublisher pubLeftVel, pubRightVel;
    private DoubleArrayPublisher pubPose;

    private static DiffTalonSubsystem instance;
    public static DiffTalonSubsystem getInstance() {
        if (instance == null) {
            if (Config.DIFF.ISNEOS) { 
                DriverStation.reportError(
                    String.format("DiffTalonSubsystem.getInstance() was called even though Config.DIFF_ISNEOS is true. RobotID: %d", Config.getRobotId()), 
                    true);
            } else {
                SubsystemChecker.subsystemConstructed(SubsystemType.DiffTalonSubsystem);
                instance = new DiffTalonSubsystem();
            }
        }

        return instance;
    }

    /** Creates a new ExampleSubsystem. */
    public DiffTalonSubsystem() {
        leftLeader = new WPI_TalonSRX(Config.CANID.DIFF_LEADER_LEFT);
        rightLeader = new WPI_TalonSRX(Config.CANID.DIFF_LEADER_RIGHT);

        // Check whether to construct a victor or a talon or nothing
        if(Config.DIFF.HAS_FOLLOWERS == true){
            if (Config.DIFF.LEFT_FOLLOWER_ISVICTOR) {
                leftFollower = new WPI_VictorSPX(Config.CANID.DIFF_FOLLOWER_LEFT);
            } else {
                leftFollower = new WPI_TalonSRX(Config.CANID.DIFF_FOLLOWER_LEFT);
            }
            if (Config.DIFF.RIGHT_FOLLOWER_ISVICTOR) {
                rightFollower = new WPI_VictorSPX(Config.CANID.DIFF_FOLLOWER_RIGHT);
            } else {
                rightFollower = new WPI_TalonSRX(Config.CANID.DIFF_FOLLOWER_RIGHT);
            }
        }
        else{
            leftFollower = null;
            rightFollower = null;
        }

        leftLeader.configFactoryDefault(Config.CAN_TIMEOUT_LONG);
        rightLeader.configFactoryDefault(Config.CAN_TIMEOUT_LONG);

        leftLeader.setInverted(Config.DIFF.LEADER_LEFT_INVERTED);
        rightLeader.setInverted(Config.DIFF.LEADER_RIGHT_INVERTED);

        leftLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        rightLeader.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        leftLeader.setSensorPhase(Config.DIFF.LEFT_SENSORPHASE);
        rightLeader.setSensorPhase(Config.DIFF.RIGHT_SENSORPHASE);

        if (leftFollower != null && rightFollower != null) {
            leftFollower.configFactoryDefault();
            rightFollower.configFactoryDefault();
            
            leftFollower.setInverted(
                Config.DIFF.FOLLOWER_LEFT_INVERTED ? InvertType.OpposeMaster : InvertType.FollowMaster);

            rightFollower.setInverted(
                Config.DIFF.FOLLOWER_RIGHT_INVERTED ? InvertType.OpposeMaster : InvertType.FollowMaster);
        }

        if (Config.CANID.PIGEON != -1) {
            if (Config.CANID.PIGEON == Config.CANID.DIFF_FOLLOWER_LEFT && leftFollower != null) 
                pigeon = new PigeonIMU((WPI_TalonSRX) leftFollower);
            else {
                pigeon = new PigeonIMU(Config.CANID.PIGEON);
            }
        }

        diffDrive = new DifferentialDrive(leftLeader, rightLeader);

        poseEstimator = new DifferentialDrivePoseEstimatorExposed(
            Config.kDriveKinematics, 
            getRawGyroHeading(), 
            getLeftDistance(), 
            getRightDistance(), 
            new Pose2d()
        );

        limelight = new LL3DApriltags("limelight");

        NetworkTable table = NetworkTableInstance.getDefault().getTable("Drivetrain");
        pubLeftVel = table.getDoubleTopic("LeftVelocityMPS").publish(PubSubOption.periodic(0.02));
        pubRightVel = table.getDoubleTopic("RightVelocityMPS").publish(PubSubOption.periodic(0.02));
        pubPose = table.getDoubleArrayTopic("EstimatedPose").publish(PubSubOption.periodic(0.02));

    }

    @Override
    public void periodic() {
        poseEstimator.update(
            getRawGyroHeading(), 
            getLeftDistance(), 
            getRightDistance()
        );

        limelight.update();

        pubPose.accept(AdvantageUtil.deconstruct(poseEstimator.getEstimatedPosition()));
    }

    public void newVisionMeasurement(Pose2d pose, double timestamp) {
        if (!disableVisionFeedback) {
            poseEstimator.addVisionMeasurement(pose, timestamp);
        }
    }

    public void disableVisionFeedback(boolean disable) {
        disableVisionFeedback = disable;
    }

    public CommandBase getToggleVisionFeedbackCommand() {
        return Commands.runOnce(
            () -> disableVisionFeedback = !disableVisionFeedback);
    }

    public Optional<Pose2d> getPoseAtTimestamp(double timestamp) {
        return poseEstimator.getPoseAtTimestamp(timestamp);
    }
    
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            getRawGyroHeading(), 
            getLeftDistance(), 
            getRightDistance(), 
            pose
        );
    }

    public void stopMotors() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();

        if(leftFollower != null) {
            leftFollower.neutralOutput();
        }
        if(rightFollower != null){
            rightFollower.neutralOutput();
        }
    }

    
    /**
     * Set the {@link NeutralMode} of the motors.
     * 
     * @param mode Desired NeutralMode.
     */
    public void setNeutralMode(NeutralMode mode) {
        leftLeader.setNeutralMode(mode);
        rightLeader.setNeutralMode(mode);
        if(leftFollower != null){
            leftFollower.setNeutralMode(mode);
        }
        if(rightFollower != null){
            rightFollower.setNeutralMode(mode);
        }
    }

    /**
     * Motor control method for arcade drive.
     * 
     * @param forwardVal The forward value
     * @param rotateVal The rotate value
     */
    public void arcadeDrive(double forwardVal, double rotateVal) {
        diffDrive.arcadeDrive(forwardVal, rotateVal, false);
    }

    /**
     * Get the fused heading from the pigeon
     * 
     * @return Heading of the robot in degrees
     */
    private Rotation2d getRawGyroHeading() {
        return Rotation2d.fromDegrees(pigeon.getFusedHeading());
    }

    /**
     * Get the encoder data in meters
     * 
     * @return Distance of the left encoder in meters
     */
    private double getLeftDistance() {
        return CTREUnits.talonPositionToMeters(leftLeader.getSelectedSensorPosition(), Config.drivetrainWheelDiameter);
    }

    /**
     * Get the encoder data in meters
     * 
     * @return Distance of the right encoder in meters
     */
    private double getRightDistance() {
        return CTREUnits.talonPositionToMeters(rightLeader.getSelectedSensorPosition(), Config.drivetrainWheelDiameter);
    }

    public double getLeftVelocity() {
        return CTREUnits.talonVelocityToMetersPerSecond(leftLeader.getSelectedSensorVelocity(), Config.drivetrainWheelDiameter);
    }

    public double getRightVelocity() {
        return CTREUnits.talonVelocityToMetersPerSecond(rightLeader.getSelectedSensorVelocity(), Config.drivetrainWheelDiameter);
    }

    /**
     * Get the measured velocity of the left and right 
     * wheels in meters per second.
     * 
     * @return The speeds of the wheels in m/s.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        double leftSpeed = getLeftVelocity();
        double rightSpeed = getRightVelocity();

        pubLeftVel.accept(leftSpeed);
        pubRightVel.accept(rightSpeed);

        return new DifferentialDriveWheelSpeeds(
            leftSpeed,
            rightSpeed
        );
    }

    public void setWheelVoltages(double leftVoltage, double rightVoltage) {
        leftLeader.setVoltage(leftVoltage);
        rightLeader.setVoltage(rightVoltage);
    }
}
