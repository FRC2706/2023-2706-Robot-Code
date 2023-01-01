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
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib3512.math.OnboardModuleState;
import frc.lib3512.util.CANCoderUtil;
import frc.lib3512.util.CANSparkMaxUtil;
import frc.lib3512.util.CANCoderUtil.CCUsage;
import frc.lib3512.util.CANSparkMaxUtil.Usage;
import frc.robot.config.Config;

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
    private NetworkTableEntry desiredSpeedEntry;
    private NetworkTableEntry desiredAngleEntry;
    private NetworkTableEntry currentSpeedEntry;
    private NetworkTableEntry currentAngleEntry;
    private NetworkTableEntry speedError;
    private NetworkTableEntry angleError;
    private double m_encoderOffset;
    private NetworkTableEntry canCoderEntry;


    SimpleMotorFeedforward feedforward = 
        new SimpleMotorFeedforward(Config.Swerve.driveKS, Config.Swerve.driveKV, Config.Swerve.driveKA);
    /**
     * Constructs a SwerveModule.
     */
    public SwerveModule(int driveCanID, boolean driveInverted, int turningCanID, boolean turningInverted, int encoderCanID, double encoderOffset, String ModuleName) {

        // CODE: Construct both CANSparkMax objects and set all the nessecary settings (CONSTANTS from Config or from the parameters of the constructor)
        m_encoderOffset = encoderOffset;
        m_driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_driveMotor, Usage.kVelocityOnly);
        m_driveMotor.setInverted(driveInverted);
        m_driveMotor.setIdleMode(Config.Swerve.defaultDriveIdleMode);
        m_driveMotor.setSmartCurrentLimit(Config.Swerve.driveCurrentLimit);
        m_driveMotor.enableVoltageCompensation(Config.Swerve.driveVoltComp);
        m_driveMotor.burnFlash();

        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setP(Config.Swerve.fluid_drive_kP.get());
        m_drivePIDController.setI(Config.Swerve.fluid_drive_kI.get());
        m_drivePIDController.setD(Config.Swerve.fluid_drive_kD.get());
        m_drivePIDController.setIZone(Config.Swerve.fluid_drive_kIZone.get());
        m_drivePIDController.setFF(Config.Swerve.fluid_drive_kFF.get());   
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_driveMotor, Usage.kPositionOnly);
        
        m_turningMotor = new CANSparkMax(turningCanID, MotorType.kBrushless);
        m_turningPIDController = m_turningMotor.getPIDController();
        m_turningMotor.enableVoltageCompensation(Config.Swerve.steeringVoltComp);

        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(Config.Swerve.drivetrainEncoderConstant);
        m_driveEncoder.setPosition(0.0);

        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.setInverted(turningInverted);
        m_turningMotor.setIdleMode(Config.Swerve.defaultSteeringIdleMode);

        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoder.setPositionConversionFactor(Config.Swerve.turningEncoderConstant);

        m_turningPIDController.setP(Config.Swerve.fluid_steering_kP.get());
        m_turningPIDController.setI(Config.Swerve.fluid_steering_kI.get());
        m_turningPIDController.setD(Config.Swerve.fluid_steering_kD.get());
        m_turningPIDController.setIZone(Config.Swerve.fluid_steering_kIZone.get());
        m_turningPIDController.setFF(Config.Swerve.fluid_steering_kFF.get());
        m_turningMotor.burnFlash();

        String tableName = "Swerve Chassis/SwerveModule" + ModuleName;
        swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);

        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();
        encoderConfiguration.initializationStrategy = 
            SensorInitializationStrategy.BootToAbsolutePosition;
        encoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;
        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.magnetOffsetDegrees = 0;
        encoderConfiguration.sensorDirection = direction == Direction.CLOCKWISE;

        encoder = new CANCoder(encoderCanID);

        encoder.configFactoryDefault();
        CANCoderUtil.setCANCoderBusUsage(encoder, CCUsage.kMinimal);
        encoder.configAllSettings(encoderConfiguration);
        
        desiredSpeedEntry = swerveModuleTable.getEntry("Desired speed (m/s)");
        desiredAngleEntry = swerveModuleTable.getEntry("Desired angle (radians)");
        currentSpeedEntry = swerveModuleTable.getEntry("Current speed (m/s)");
        currentAngleEntry = swerveModuleTable.getEntry("Current angle (radians)");
        speedError = swerveModuleTable.getEntry("speed Error");
        angleError = swerveModuleTable.getEntry("angle Error"); 
        canCoderEntry = swerveModuleTable.getEntry("CanCoder");

        updateSteeringFromCanCoder();

        lastAngle = getState().angle.getRadians();
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
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        Rotation2d angle = getSteeringAngle();
        double velocity = getVelocity();

        desiredState = 
            OnboardModuleState.optimize(
                desiredState, angle);
        

        if (isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond/Config.Swerve.kMaxAttainableWheelSpeed;
            m_driveMotor.set(percentOutput);
        } else{
            m_drivePIDController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
        // CODE: Pass the velocity (which is in meters per second) to velocity PID on drive SparkMax. (VelocityConversionFactor set so SparkMax wants m/s)
        

        double newAngle = 
            (Math.abs(desiredState.speedMetersPerSecond) <= (Config.Swerve.kMaxAttainableWheelSpeed *0.01))
                ? lastAngle
                :desiredState.angle.getRadians();
        // CODE: Pass the angle (which is in radians) to position PID on steering SparkMax. (PositionConversionFactor set so SparkMax wants radians)
        m_turningPIDController.setReference(newAngle, ControlType.kPosition);

        lastAngle = newAngle;

        desiredSpeedEntry.setDouble(desiredState.speedMetersPerSecond);
        desiredAngleEntry.setDouble(desiredState.angle.getDegrees());
        currentSpeedEntry.setDouble(velocity);
        currentAngleEntry.setDouble(angle.getDegrees());
        speedError.setDouble(desiredState.speedMetersPerSecond - velocity);
        angleError.setDouble(desiredState.angle.getDegrees() - angle.getDegrees());
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
     * Gets a reading from the Lamprey and updates the SparkMax encoder (interal NEO encoder).
     * This is specific to Swerge. Other methods need to be written for other hardware.
     */
    public void updateSteeringFromCanCoder() {
        double angle = Math.toRadians(m_encoderOffset + encoder.getAbsolutePosition());
        m_turningEncoder.setPosition(angle);

    }

    public void updatePIDValues(){
        m_drivePIDController.setP(Config.Swerve.fluid_drive_kP.get());
        m_drivePIDController.setI(Config.Swerve.fluid_drive_kI.get());
        m_drivePIDController.setD(Config.Swerve.fluid_drive_kD.get());
        m_drivePIDController.setIZone(Config.Swerve.fluid_drive_kIZone.get());
        m_drivePIDController.setFF(Config.Swerve.fluid_drive_kFF.get());

        m_turningPIDController.setP(Config.Swerve.fluid_steering_kP.get());
        m_turningPIDController.setI(Config.Swerve.fluid_steering_kI.get());
        m_turningPIDController.setD(Config.Swerve.fluid_steering_kD.get());
        m_turningPIDController.setIZone(Config.Swerve.fluid_steering_kIZone.get());
        m_turningPIDController.setFF(Config.Swerve.fluid_steering_kFF.get());

        feedforward = new SimpleMotorFeedforward(Config.Swerve.fluid_kS.get(), Config.Swerve.fluid_kV.get(), Config.Swerve.fluid_kA.get());
    
    }

    public void updateNT(){
        canCoderEntry.setDouble(Math.toDegrees(m_turningEncoder.getPosition()));
    }

    /**
     * Standard stop motors method for every subsystem.
     */
    public void stopMotors() {
        // CODE: Call the stopMotors method in the CANSparkMax (provided with all WPILib motor controller objects)
        m_driveMotor.stopMotor();
        m_turningMotor.stopMotor();
    }

    private Direction direction = Direction.COUNTER_CLOCKWISE;
    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}
