// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.config.Config;

public class ArmSubsystem extends SubsystemBase {

  private static final MotorType motorType = MotorType.kBrushless;
  private static final SparkMaxAbsoluteEncoder.Type encAbsType = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
  
  public CANCoder m_absoluteTopArmEncoder;
  public CANCoder m_absoluteBottomArmEncoder;


  private static ArmSubsystem instance = null;


  public final CANSparkMax m_topArm;
  public final CANSparkMax m_bottomArm;
  public SparkMaxPIDController m_pidControllerTopArm;
  public SparkMaxPIDController m_pidControllerBottomArm;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_topArm = new CANSparkMax(Config.CANID.TOP_ARM_SPARK_CAN_ID, motorType);
    m_bottomArm = new CANSparkMax(Config.CANID.BOTTOM_ARM_SPARK_CAN_ID, motorType);
    CANCoderConfiguration config = new CANCoderConfiguration();
    m_topArm.restoreFactoryDefaults();
    m_bottomArm.restoreFactoryDefaults();

    // set units of the CANCoder to radians, with velocity being radians per second
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;

    m_absoluteTopArmEncoder = new CANCoder(16);
    m_absoluteBottomArmEncoder = new CANCoder(17);

    m_absoluteTopArmEncoder.configAllSettings(config);
    m_absoluteBottomArmEncoder.configAllSettings(config);

    m_pidControllerTopArm = m_topArm.getPIDController();
    m_pidControllerBottomArm = m_bottomArm.getPIDController();

    // PID coefficients (probably need to change values and put the values in config)
    kP = 0.02;
    kI = 0;
    kD = 2.4;
    kIz = 0;
    kFF = 0.1;
    kMaxOutput = 0.5;
    kMinOutput = -0.5;

    // setting PID coefficients for top arm
    m_pidControllerTopArm.setP(kP);
    m_pidControllerTopArm.setI(kI);
    m_pidControllerTopArm.setD(kD);
    m_pidControllerTopArm.setIZone(kIz);
    m_pidControllerTopArm.setFF(kFF);
    m_pidControllerTopArm.setOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerTopArm.setSmartMotionMaxAccel(Math.PI/2, 0); // maybe accel and velocity should be in rpm instead of radians/second ?
    m_pidControllerTopArm.setSmartMotionMaxVelocity(Math.PI/2, 0);
    
    m_bottomArm.getEncoder().setPositionConversionFactor(2*Math.PI / Config.Arm.NEO_GEAR_RATIO);
    m_topArm.getEncoder().setPositionConversionFactor(2*Math.PI / Config.Arm.NEO_GEAR_RATIO);

    // setting PID coefficients for bottom arm
    m_pidControllerBottomArm.setP(kP);
    m_pidControllerBottomArm.setI(kI);
    m_pidControllerBottomArm.setD(kD);
    m_pidControllerBottomArm.setIZone(kIz);
    m_pidControllerBottomArm.setFF(kFF);
    m_pidControllerBottomArm.setOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerBottomArm.setSmartMotionMaxAccel(Math.PI/2, 0);
    m_pidControllerBottomArm.setSmartMotionMaxVelocity(Math.PI/2, 0);

  }

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
      instance = new ArmSubsystem();
    }
    return instance;
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double[] calculateAngle(double L1, double L2, double x, double z) {
    double zx = (Math.pow(x,2)+Math.pow(z,2));
    //angle2 --> top arm
    double angle2 = Math.acos((zx-Math.pow(L1,2)-Math.pow(L2,2))/(-2*L1*L2)); //gives angle in radians
    //angle1 --> bottom arm
    double angle1 = (Math.atan2(z,x)+Math.acos((Math.pow(L2,2)-zx-Math.pow(L1,2))/(-2*Math.sqrt(zx)*L1))); // gives angle in radians
    double[] angles = {angle1,angle2};
    return angles;
  }
  public double setx(double drivetrain_x,double Node_x){
    double x = Node_x - drivetrain_x;
    return x;
  }
  public double getAngularDistance(double angle, double gearRatio) {
    return angle / gearRatio;
  }
  public void setJoint1(double angle) {
    m_pidControllerTopArm.setReference(angle, ControlType.kSmartMotion); //this is only for testing
    // m_pidControllerBottomArm.setReference(angle * (1/(2*Math.PI)), ControlType.kPosition); 
  }
  public void setJoint2(double angle) {
    m_pidControllerTopArm.setReference(angle, ControlType.kSmartMotion, 0, calculateFFJoint2()); // unit conversion 1 radian --> 1/2pi
  }
  public void setDefault(double[] angle) {
    m_pidControllerTopArm.setReference(angle[0], ControlType.kSmartMotion); 
    // m_pidControllerBottomArm.setReference(angle[1] * (1/(2*Math.PI)), ControlType.kPosition); - only used when 2 motors are involved
  }
  public void resetEncoder() {
    m_topArm.getEncoder().setPosition(0);
    // m_bottomArm.getEncoder().setPosition(0);
  }
  private double calculateFFJoint2() {
    return 0;
  }


}
