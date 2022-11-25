// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;

public class SwerveModule {

    // CODE: Prepare 2 variables for both SparkMaxs, use the object called CANSparkMax
    private CANSparkMax m_driveMotor;
    private CANSparkMax m_turningMotor;
    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_turningEncoder;
    private SparkMaxPIDController m_drivePIDController;
    private SparkMaxPIDController m_turningPIDController;
    private CANCoder encoder;
    private NetworkTable swerveModuleTable;
    private NetworkTableEntry desiredSpeedEntry;
    private NetworkTableEntry desiredAngleEntry;
    private NetworkTableEntry currentSpeedEntry;
    private NetworkTableEntry currentAngleEntry;
    private NetworkTableEntry speedError;
    private NetworkTableEntry angleError;

    /**
     * Constructs a SwerveModule.
     */
    public SwerveModule(int driveCanID, boolean driveInverted, int turningCanID, boolean turningInverted, int encoderCanID, double encoderOffset, String ModuleName) {

        // CODE: Construct both CANSparkMax objects and set all the nessecary settings (CONSTANTS from Config or from the parameters of the constructor)
        m_driveMotor = new CANSparkMax(driveCanID, MotorType.kBrushless);

        m_driveMotor.restoreFactoryDefaults();
        m_driveMotor.setInverted(driveInverted);
        m_driveMotor.setIdleMode(IdleMode.kCoast);

        m_drivePIDController = m_driveMotor.getPIDController();
        m_drivePIDController.setP(Config.fluid_drive_kP.get());
        m_drivePIDController.setI(Config.fluid_drive_kI.get());
        m_drivePIDController.setD(Config.fluid_drive_kD.get());
        m_drivePIDController.setIZone(Config.fluid_drive_kIZone.get());
        m_drivePIDController.setFF(Config.fluid_drive_kFF.get());   
        
        m_turningMotor = new CANSparkMax(turningCanID, MotorType.kBrushless);
        m_turningPIDController = m_turningMotor.getPIDController();

        m_driveEncoder = m_driveMotor.getEncoder();
        m_driveEncoder.setVelocityConversionFactor(Config.drivetrainEncoderConstant);

        m_turningMotor.restoreFactoryDefaults();
        m_turningMotor.setInverted(turningInverted);
        m_turningMotor.setIdleMode(IdleMode.kCoast);

        m_turningEncoder = m_turningMotor.getEncoder();
        m_turningEncoder.setPositionConversionFactor(Config.turningEncoderConstant);

        m_turningPIDController.setP(Config.fluid_steering_kP.get());
        m_turningPIDController.setI(Config.fluid_steering_kI.get());
        m_turningPIDController.setD(Config.fluid_steering_kD.get());
        m_turningPIDController.setIZone(Config.fluid_steering_kIZone.get());
        m_turningPIDController.setFF(Config.fluid_steering_kFF.get());

        String tableName = "Swerve Chassis/SwerveModule" + ModuleName;
        swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);

        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();
        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.magnetOffsetDegrees = encoderOffset;
        encoderConfiguration.sensorDirection = direction == Direction.CLOCKWISE;

        encoder = new CANCoder(encoderCanID);
        
        desiredSpeedEntry = swerveModuleTable.getEntry("Desired speed (m/s)");
        desiredAngleEntry = swerveModuleTable.getEntry("Desired angle (radians)");
        currentSpeedEntry = swerveModuleTable.getEntry("Current speed (m/s)");
        currentAngleEntry = swerveModuleTable.getEntry("Current angle (radians)");
        speedError = swerveModuleTable.getEntry("speed Error");
        angleError = swerveModuleTable.getEntry("angle Error"); 

        updateSteeringFromCanCoder();
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
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        double measuredVelocity = getVelocity();
        Rotation2d measuredAngle = getSteeringAngle();
        double deltaAngle = desiredState.angle.getRadians() - measuredAngle.getRadians();
        //make sure deltaAngle in [0,2pi]
        deltaAngle %= 2.0*Math.PI;
        if (deltaAngle < 0.0)
        deltaAngle += 2.0*Math.PI;
        //make sure detlaAngle in [-pi, pi]
        if ( deltaAngle > Math.PI && deltaAngle <= 2*Math.PI)
        {
            deltaAngle -= 2*Math.PI;
        }
        //make sure (desiredState.angle - currentAngle) difference is [-pi, pi], which is what optimize() requires.
        Rotation2d currentAngle = new Rotation2d(- deltaAngle + desiredState.angle.getRadians() );
        //Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState updatedDesiredstate = SwerveModuleState.optimize(desiredState, currentAngle);
        double velocity = updatedDesiredstate.speedMetersPerSecond;
        //deltaAngle now is in [-pi/2, pi/2], which is the angle to be adjusted.
        deltaAngle = updatedDesiredstate.angle.minus(currentAngle).getRadians();
        double angle = (measuredAngle.getRadians() + deltaAngle);
        
        
        // CODE: Pass the velocity (which is in meters per second) to velocity PID on drive SparkMax. (VelocityConversionFactor set so SparkMax wants m/s)
        m_drivePIDController.setReference(velocity, ControlType.kVelocity);

        // CODE: Pass the angle (which is in radians) to position PID on steering SparkMax. (PositionConversionFactor set so SparkMax wants radians)
        m_turningPIDController.setReference(angle, ControlType.kPosition);

        desiredSpeedEntry.setDouble(velocity);
        desiredAngleEntry.setDouble(angle);
        currentSpeedEntry.setDouble(measuredVelocity);
        currentAngleEntry.setDouble(measuredAngle.getDegrees());
        speedError.setDouble(velocity - measuredVelocity);
        angleError.setDouble(angle - measuredAngle.getDegrees());
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
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        m_turningEncoder.setPosition(angle);

    }

    public void updatePIDValues(){
        m_drivePIDController.setP(Config.fluid_drive_kP.get());
        m_drivePIDController.setI(Config.fluid_drive_kI.get());
        m_drivePIDController.setD(Config.fluid_drive_kD.get());
        m_drivePIDController.setIZone(Config.fluid_drive_kIZone.get());
        m_drivePIDController.setFF(Config.fluid_drive_kFF.get());

        m_turningPIDController.setP(Config.fluid_steering_kP.get());
        m_turningPIDController.setI(Config.fluid_steering_kI.get());
        m_turningPIDController.setD(Config.fluid_steering_kD.get());
        m_turningPIDController.setIZone(Config.fluid_steering_kIZone.get());
        m_turningPIDController.setFF(Config.fluid_steering_kFF.get());
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
