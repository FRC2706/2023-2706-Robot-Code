// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class ArmSubsystem extends SubsystemBase {

  private static final int SparkTopArmCANID = 0;
  private static final int SparkBottomArmCANID = 0;
  private static final MotorType motorType = MotorType.kBrushless;
  private static final SparkMaxAbsoluteEncoder.Type encAbsType = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
  
  private AbsoluteEncoder m_absoluteTopArmEncoder;
  private AbsoluteEncoder m_absoluteBottomArmEncoder;


  private final CANSparkMax m_topArm;
  private final CANSparkMax m_bottomArm;
  private SparkMaxPIDController m_pidControllerTopArm;
  private SparkMaxPIDController m_pidControllerBottomArm;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_topArm = new CANSparkMax(SparkTopArmCANID, motorType);
    m_bottomArm = new CANSparkMax(SparkBottomArmCANID, motorType);
    m_topArm.restoreFactoryDefaults();
    m_bottomArm.restoreFactoryDefaults();

    m_absoluteTopArmEncoder = m_topArm.getAbsoluteEncoder(encAbsType);
    m_absoluteBottomArmEncoder = m_bottomArm.getAbsoluteEncoder(encAbsType);

    m_pidControllerTopArm = m_topArm.getPIDController();
    m_pidControllerBottomArm = m_bottomArm.getPIDController();

    m_pidControllerTopArm.setFeedbackDevice(m_absoluteTopArmEncoder);
    m_pidControllerBottomArm.setFeedbackDevice(m_absoluteBottomArmEncoder);

    // PID coefficients (probably need to change values and put the values in config)
    kP = 0.1;
    kI = 1e-4;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    // setting PID coefficients for top arm
    m_pidControllerTopArm.setP(kP);
    m_pidControllerTopArm.setI(kI);
    m_pidControllerTopArm.setD(kD);
    m_pidControllerTopArm.setIZone(kIz);
    m_pidControllerTopArm.setFF(kFF);
    m_pidControllerTopArm.setOutputRange(kMinOutput, kMaxOutput);

    // setting PID coefficients for bottom arm
    m_pidControllerBottomArm.setP(kP);
    m_pidControllerBottomArm.setI(kI);
    m_pidControllerBottomArm.setD(kD);
    m_pidControllerBottomArm.setIZone(kIz);
    m_pidControllerBottomArm.setFF(kFF);
    m_pidControllerBottomArm.setOutputRange(kMinOutput, kMaxOutput);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
