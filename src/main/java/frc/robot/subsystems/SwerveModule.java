// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib3512.math.OnboardModuleState;
import frc.lib3512.util.CANCoderUtil;
import frc.lib3512.util.CANSparkMaxUtil;
import frc.lib3512.util.CANCoderUtil.CCUsage;
import frc.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.config.Config;
import static frc.robot.ErrorCheck.errCTRE;
import static frc.robot.ErrorCheck.errREV;


public class SwerveModule {

    // CODE: Prepare 2 variables for both SparkMaxs, use the object called CANSparkMax
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turningMotor;
    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turningEncoder;
    private SparkMaxPIDController m_drivePIDController;
    private SparkMaxPIDController m_turningPIDController;
    private CANCoder encoder;
    private double lastAngle;
    private NetworkTable swerveModuleTable;
    private DoublePublisher desiredSpeedEntry;
    private DoublePublisher desiredAngleEntry;
    private DoublePublisher currentSpeedEntry;
    private DoublePublisher currentAngleEntry;
    private DoublePublisher speedError;
    private DoublePublisher angleError;
    private DoublePublisher desiredAngle360Range;
    private DoublePublisher currentAngle360Range;
    private double m_encoderOffset;
    private DoublePublisher canCoderEntry;
    private DoubleSubscriber sub_offset;


    SimpleMotorFeedforward feedforward = 
        new SimpleMotorFeedforward(Config.Swerve.driveKS, Config.Swerve.driveKV, Config.Swerve.driveKA);
    /**
     * Constructs a SwerveModule.
     */
    public SwerveModule(int driveCanID, boolean driveInverted, int turningCanID, boolean turningInverted, int encoderCanID, double encoderOffset, String ModuleName) {

        // CODE: Construct both CANSparkMax objects and set all the nessecary settings (CONSTANTS from Config or from the parameters of the constructor)
        m_driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);

        errREV(m_driveMotor.restoreFactoryDefaults());
        m_driveMotor.setInverted(driveInverted);
        errREV(m_driveMotor.setIdleMode(Config.Swerve.defaultDriveIdleMode));
        errREV(m_driveMotor.setSmartCurrentLimit(Config.Swerve.driveCurrentLimit));
        errREV(m_driveMotor.enableVoltageCompensation(Config.Swerve.driveVoltComp));

        m_drivePIDController = m_driveMotor.getPIDController();
        errREV(m_drivePIDController.setP(Config.Swerve.sub_drive_kP.get()));
        errREV(m_drivePIDController.setI(Config.Swerve.sub_drive_kI.get()));
        errREV(m_drivePIDController.setD(Config.Swerve.sub_drive_kD.get()));
        errREV(m_drivePIDController.setIZone(Config.Swerve.sub_drive_kIZone.get()));
        errREV(m_drivePIDController.setFF(Config.Swerve.sub_drive_kFF.get()));   
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_driveMotor, Usage.kAll);
        
        m_turningMotor = new CANSparkMax(turningCanID, MotorType.kBrushless);
        errREV(m_turningMotor.restoreFactoryDefaults());
        m_turningPIDController = m_turningMotor.getPIDController();
        errREV(m_turningMotor.enableVoltageCompensation(Config.Swerve.steeringVoltComp));
        errREV(m_turningMotor.setSmartCurrentLimit(Config.Swerve.steeringCurrentLimit));

        m_driveEncoder = m_driveMotor.getEncoder();
        errREV(m_driveEncoder.setPositionConversionFactor(Config.Swerve.drivePositionConversionFactor));
        errREV(m_driveEncoder.setVelocityConversionFactor(Config.Swerve.driveVelocityConversionFactor));
        errREV(m_driveEncoder.setPosition(0.0));

        
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_turningMotor, Usage.kPositionOnly);
        m_turningMotor.setInverted(turningInverted);
        errREV(m_turningMotor.setIdleMode(Config.Swerve.defaultSteeringIdleMode));

        m_turningEncoder = m_turningMotor.getEncoder();
        errREV(m_turningEncoder.setPositionConversionFactor(Config.Swerve.turningEncoderConstant));

        errREV(m_turningPIDController.setP(Config.Swerve.sub_steering_kP.get()));
        errREV(m_turningPIDController.setI(Config.Swerve.sub_steering_kI.get()));
        errREV(m_turningPIDController.setD(Config.Swerve.sub_steering_kD.get()));
        errREV(m_turningPIDController.setIZone(Config.Swerve.sub_steering_kIZone.get()));
        errREV(m_turningPIDController.setFF(Config.Swerve.sub_steering_kFF.get()));

        errREV(m_driveMotor.burnFlash());
        errREV(m_turningMotor.burnFlash());

        String tableName = "SwerveChassis/SwerveModule" + ModuleName;
        swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);

        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();
        encoderConfiguration.initializationStrategy = 
            SensorInitializationStrategy.BootToAbsolutePosition;
        encoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.magnetOffsetDegrees = 0;
        encoderConfiguration.sensorDirection = direction == Direction.CLOCKWISE;

        encoder = new CANCoder(encoderCanID);

        errCTRE(encoder.configFactoryDefault());
        CANCoderUtil.setCANCoderBusUsage(encoder, CCUsage.kMinimal);
        errCTRE(encoder.configAllSettings(encoderConfiguration));
        
        desiredSpeedEntry = swerveModuleTable.getDoubleTopic("Desired speed (mps)").publish();
        desiredAngleEntry = swerveModuleTable.getDoubleTopic("Desired angle (deg)").publish();
        currentSpeedEntry = swerveModuleTable.getDoubleTopic("Current speed (mps)").publish();
        currentAngleEntry = swerveModuleTable.getDoubleTopic("Current angle (deg)").publish();
        speedError = swerveModuleTable.getDoubleTopic("Speed error (mps)").publish();
        angleError = swerveModuleTable.getDoubleTopic("Angle error (deg)").publish(); 
        canCoderEntry = swerveModuleTable.getDoubleTopic("CanCoder").publish();
        desiredAngle360Range = swerveModuleTable.getDoubleTopic("Desired angle 360 range").publish();
        currentAngle360Range = swerveModuleTable.getDoubleTopic("Current angle 360 range").publish();
        sub_offset = swerveModuleTable.getDoubleTopic("Current angle 360 range").subscribe(encoderOffset);
        
        updateSteeringFromCanCoder();

        resetLastAngle();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {   
        return new SwerveModuleState(getVelocity(), getSteeringAngle());
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModulePosition getModulePosition() {   
        return new SwerveModulePosition(getDrivePosition(), getSteeringAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop, boolean useAntiJitter) {
        Rotation2d angle = getSteeringAngle();
        double velocity = getVelocity();
        double newAngle;

        desiredState = 
            OnboardModuleState.optimize(
                desiredState, angle);
        

        if (isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond/Config.Swerve.kMaxAttainableWheelSpeed;
            m_driveMotor.set(percentOutput);
        } else{
            errREV(m_drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond)));
        }
        // CODE: Pass the velocity (which is in meters per second) to velocity PID on drive SparkMax. (VelocityConversionFactor set so SparkMax wants m/s)
        newAngle = 
        (Math.abs(desiredState.speedMetersPerSecond) >= (Config.Swerve.kMaxAttainableWheelSpeed *0.001) && useAntiJitter)
            ? desiredState.angle.getRadians()
            :lastAngle;
    
        // CODE: Pass the angle (which is in radians) to position PID on steering SparkMax. (PositionConversionFactor set so SparkMax wants radians)
        errREV(m_turningPIDController.setReference(newAngle, ControlType.kPosition));

        lastAngle = newAngle;

        desiredSpeedEntry.accept(desiredState.speedMetersPerSecond);
        desiredAngleEntry.accept(desiredState.angle.getDegrees());
        currentSpeedEntry.accept(velocity);
        currentAngleEntry.accept(angle.getDegrees());
        speedError.accept(desiredState.speedMetersPerSecond - velocity);
        angleError.accept(desiredState.angle.getDegrees() - angle.getDegrees());
        desiredAngle360Range.accept(MathUtil.inputModulus(desiredState.angle.getDegrees(), -180.0, 180.0));
        currentAngle360Range.accept(MathUtil.inputModulus(angle.getDegrees(), -180.0, 180.0));
        NetworkTableInstance.getDefault().flush();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {    
        setDesiredState(desiredState, isOpenLoop, true);
    }

    public void resetLastAngle() {
        lastAngle = getState().angle.getRadians();
    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * 
     * @return meters per second of the wheel
     */
    public double getDrivePosition() {

        // CODE: Read encoder velocity from drive SparkMax and return m/s. (VelocityConversionFactor set so SparkMax returns m/s))
        
        return m_driveEncoder.getPosition();
    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * 
     * @return meters per second of the wheel
     */
    public double getVelocity() {

        // CODE: Read encoder velocity from drive SparkMax and return m/s. (VelocityConversionFactor set so SparkMax returns m/s))
        
        return m_driveEncoder.getVelocity();
    }

    /**
     * Returns the angle the wheel is pointing in a Rotation2d.
     * 
     * @return angle of the wheel as a Rotation2d
     */
    public Rotation2d getSteeringAngle() {

        // CODE: Read encoder position from steering SparkMax and return Rotation2d.
        // The PositionConversionFactor is set so SparkMax returns radians, the default constructor of Rotation2d wants radians.

        return new Rotation2d(m_turningEncoder.getPosition());
    }

    /**
     * Returns the Cancoder + offset.
     * @return radians
     */
    public double getCancoder() {
        return Math.toRadians(encoder.getAbsolutePosition() + sub_offset.get());
    }

    /**
     * Gets a reading from the CanCoder and updates the SparkMax encoder (interal NEO encoder).
     * This is specific to Swerge. Other methods need to be written for other hardware.
     */
    public void updateSteeringFromCanCoder() {
        errREV(m_turningEncoder.setPosition(getCancoder()));

    }

    public void updatePIDValues(){
        errREV(m_drivePIDController.setP(Config.Swerve.sub_drive_kP.get()));
        errREV(m_drivePIDController.setI(Config.Swerve.sub_drive_kI.get()));
        errREV(m_drivePIDController.setD(Config.Swerve.sub_drive_kD.get()));
        errREV(m_drivePIDController.setIZone(Config.Swerve.sub_drive_kIZone.get()));
        errREV(m_drivePIDController.setFF(Config.Swerve.sub_drive_kFF.get()));

        errREV(m_turningPIDController.setP(Config.Swerve.sub_steering_kP.get()));
        errREV(m_turningPIDController.setI(Config.Swerve.sub_steering_kI.get()));
        errREV(m_turningPIDController.setD(Config.Swerve.sub_steering_kD.get()));
        errREV(m_turningPIDController.setIZone(Config.Swerve.sub_steering_kIZone.get()));
        errREV(m_turningPIDController.setFF(Config.Swerve.sub_steering_kFF.get()));

        feedforward = new SimpleMotorFeedforward(Config.Swerve.sub_kS.get(), Config.Swerve.sub_kV.get(), Config.Swerve.sub_kA.get());
    
    }

    public void updateNT(){
        canCoderEntry.accept(Math.toDegrees(m_turningEncoder.getPosition()));
    }

    /**
     * Standard stop motors method for every subsystem.
     */
    public void stopMotors() {
        // CODE: Call the stopMotors method in the CANSparkMax (provided with all WPILib motor controller objects)
        m_driveMotor.stopMotor();
        m_turningMotor.stopMotor();
    }

    /**
     * Checks if the Neo encoder is synced with the CanCoder
     * NOTE: This only works before the first enable since optimizing messes it up
     * 
     * @return Whether the encoders are synced or not
     */
    public boolean areSteeringEncodersSynced() {
        return Math.abs(getCancoder() - getSteeringAngle().getRadians()) < 0.0017;
    }

    private Direction direction = Direction.COUNTER_CLOCKWISE;
    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
