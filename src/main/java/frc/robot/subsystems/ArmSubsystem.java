// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ProfileExternalPIDController;
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
  public ProfileExternalPIDController m_topPID;
  public ProfileExternalPIDController m_bottomPID;
  private RelativeEncoder m_bottomEncoder;
  private RelativeEncoder m_topEncoder;
  public final double kP = 0.2;
  public final double kI = 0;
  public final double kD = 0; 
  public final double kIz = 0;
  public final double kFF = 0.4;
  public final double kMaxOutput = 0.5;
  public final double kMinOutput = -0.5;

  // for top arm
  private final String m_tuningTableTop = "Arm/TopArmTuning";
  private final String m_dataTableTop = "Arm/TopArmData";

  //for bottom arm
  private final String m_tuningTableBottom = "Arm/BottomArmTuning";
  private final String m_dataTableBottom = "Arm/BottomArmData";

  // network table entries for top arm
  private DoubleEntry m_topArmPSubs;
  private DoubleEntry m_topArmISubs;
  private DoubleEntry m_topArmDSubs;
  private DoubleEntry m_topArmIzSubs;
  private DoubleEntry m_topArmFFSubs;
  private DoublePublisher m_topArmPosPub;
  private DoublePublisher m_topArmSetpointPub;   
  private DoublePublisher m_topArmVelPub;
  private DoubleEntry m_topArmVoltsAtHorizontal;
  private DoublePublisher m_topArmFFTestingVolts;

  // network table entries for bottom arm
  private DoubleEntry m_bottomArmPSubs;
  private DoubleEntry m_bottomArmISubs;
  private DoubleEntry m_bottomArmDSubs;
  private DoubleEntry m_bottomArmIzSubs;
  private DoubleEntry m_bottomArmFFSubs;
  private DoublePublisher m_bottomArmPosPub;
  private DoublePublisher m_bottomArmSetpointPub;   
  private DoublePublisher m_bottomArmVelPub;
  private DoubleEntry m_bottomArmVoltsAtHorizontal;
  private DoublePublisher m_bottomArmFFTestingVolts;

  public static ArmSubsystem getInstance() {
    if (instance == null) {
      SubsystemChecker.subsystemConstructed(SubsystemType.ArmSubsystem);
      instance = new ArmSubsystem();
    }
    return instance;
}

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_topArm = new CANSparkMax(Config.CANID.TOP_ARM_SPARK_CAN_ID, motorType);
    m_bottomArm = new CANSparkMax(Config.CANID.BOTTOM_ARM_SPARK_CAN_ID, motorType);
    CANCoderConfiguration config = new CANCoderConfiguration();
    m_topArm.restoreFactoryDefaults();
    m_bottomArm.restoreFactoryDefaults();
    m_topArm.setSmartCurrentLimit(40);
    m_bottomArm.setSmartCurrentLimit(40);
    m_topArm.setInverted(false);
    m_bottomArm.setInverted(false);
    m_topArm.setIdleMode(IdleMode.kBrake);
    m_bottomArm.setIdleMode(IdleMode.kBrake);
    

    // set units of the CANCoder to radians, with velocity being radians per second
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;

    m_absoluteTopArmEncoder = new CANCoder(16);
    m_absoluteBottomArmEncoder = new CANCoder(17);

    m_absoluteTopArmEncoder.configAllSettings(config);
    m_absoluteBottomArmEncoder.configAllSettings(config);

    m_pidControllerTopArm = m_topArm.getPIDController();
    m_pidControllerBottomArm = m_bottomArm.getPIDController();
    updatePIDSettings();
    m_topPID = new ProfileExternalPIDController(new Constraints(0, 0));
    m_bottomPID = new ProfileExternalPIDController(new Constraints(0, 0));
    setConstraints(true);

    m_bottomEncoder = m_bottomArm.getEncoder();
    m_topEncoder = m_topArm.getEncoder();
    
    m_bottomEncoder.setPositionConversionFactor(Config.Arm.armPositionConversionFactor);
    m_topEncoder.setPositionConversionFactor(Config.Arm.armPositionConversionFactor);
    m_topEncoder.setVelocityConversionFactor(Config.Arm.armVelocityConversionFactor);
    m_bottomEncoder.setVelocityConversionFactor(Config.Arm.armVelocityConversionFactor);

    NetworkTable topArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableTop);
    m_topArmPSubs = topArmTuningTable.getDoubleTopic("P").getEntry(kP);
    m_topArmISubs = topArmTuningTable.getDoubleTopic("I").getEntry(kI);
    m_topArmDSubs = topArmTuningTable.getDoubleTopic("D").getEntry(kD);
    m_topArmIzSubs = topArmTuningTable.getDoubleTopic("IZone").getEntry(kIz);
    m_topArmFFSubs = topArmTuningTable.getDoubleTopic("FF").getEntry(kFF);
    
    NetworkTable bottomArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableBottom);
    m_bottomArmPSubs = bottomArmTuningTable.getDoubleTopic("P").getEntry(kP);
    m_bottomArmISubs = bottomArmTuningTable.getDoubleTopic("I").getEntry(kI);
    m_bottomArmDSubs = bottomArmTuningTable.getDoubleTopic("D").getEntry(kD);
    m_bottomArmIzSubs = bottomArmTuningTable.getDoubleTopic("IZone").getEntry(kIz);
    m_bottomArmFFSubs = bottomArmTuningTable.getDoubleTopic("FF").getEntry(kFF);

    if (m_topArmPSubs.getAtomic().timestamp == 0) {
        m_topArmFFSubs.accept(kFF);
        m_topArmPSubs.accept(kP);
        m_topArmISubs.accept(kI);
        m_topArmDSubs.accept(kD);
        m_topArmIzSubs.accept(kIz);
    }

    if (m_bottomArmPSubs.getAtomic().timestamp == 0) {
      m_bottomArmFFSubs.accept(kFF);
      m_bottomArmPSubs.accept(kP);
      m_bottomArmISubs.accept(kI);
      m_bottomArmDSubs.accept(kD);
      m_bottomArmIzSubs.accept(kIz);
  }

    NetworkTable topArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTableTop);
    m_topArmPosPub = topArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_topArmSetpointPub = topArmDataTable.getDoubleTopic("SetpointAngle").publish(PubSubOption.periodic(0.02));
    m_topArmVelPub = topArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));
    m_topArmVoltsAtHorizontal = topArmDataTable.getDoubleTopic("VoltsAtHorizontal").getEntry(0);
    m_topArmVoltsAtHorizontal.accept(0.9);

    m_topArmFFTestingVolts = topArmDataTable.getDoubleTopic("VoltageSetInFFTesting").publish();

    NetworkTable bottomArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTableBottom);
    m_bottomArmPosPub = bottomArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_bottomArmSetpointPub = bottomArmDataTable.getDoubleTopic("SetpointAngle").publish(PubSubOption.periodic(0.02));
    m_bottomArmVelPub = bottomArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));
    m_bottomArmVoltsAtHorizontal = bottomArmDataTable.getDoubleTopic("VoltsAtHorizontal").getEntry(0);
    m_bottomArmVoltsAtHorizontal.accept(0.9);

    m_bottomArmFFTestingVolts = bottomArmDataTable.getDoubleTopic("VoltageSetInFFTesting").publish();

  }

  public void updatePIDSettings() {
    // setting PID constants for top spark max
    m_pidControllerTopArm.setFF(m_topArmFFSubs.get());
    m_pidControllerTopArm.setP(m_topArmPSubs.get());
    m_pidControllerTopArm.setI(m_topArmISubs.get());
    m_pidControllerTopArm.setD(m_topArmDSubs.get());
    m_pidControllerTopArm.setIZone(m_topArmIzSubs.get()); 
    m_pidControllerTopArm.setOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerTopArm.setSmartMotionMaxAccel(Math.PI/6, 0);
    m_pidControllerTopArm.setSmartMotionMaxVelocity(Math.PI/6, 0);

    // setting PID constants for bottom spark max
    m_pidControllerBottomArm.setFF(m_bottomArmFFSubs.get());
    m_pidControllerBottomArm.setP(m_bottomArmPSubs.get());
    m_pidControllerBottomArm.setI(m_bottomArmISubs.get());
    m_pidControllerBottomArm.setD(m_bottomArmDSubs.get());
    m_pidControllerBottomArm.setIZone(m_bottomArmIzSubs.get()); 
    m_pidControllerBottomArm.setOutputRange(kMinOutput, kMaxOutput);
    m_pidControllerBottomArm.setSmartMotionMaxAccel(Math.PI/6, 0);
    m_pidControllerBottomArm.setSmartMotionMaxVelocity(Math.PI/6, 0);
}

  @Override
  public void periodic() {
    m_topArmPosPub.accept(Math.toDegrees(m_topEncoder.getPosition()));
    m_topArmVelPub.accept(m_topEncoder.getVelocity());
    m_bottomArmPosPub.accept(Math.toDegrees(m_bottomEncoder.getPosition()));
    m_bottomArmVelPub.accept(m_bottomEncoder.getVelocity());
    if (m_topEncoder.getVelocity() > 2) {
        m_topPID.setConstraints(new Constraints(2.2, Math.PI * 0.9));
    }
    if (m_bottomEncoder.getVelocity() > 2) {
      m_bottomPID.setConstraints(new Constraints(2.2, Math.PI * 0.9));
  }
  }

  public void setConstraints(boolean slowerAcceleration) {
    if (slowerAcceleration) {
        m_topPID.setConstraints(new Constraints(2.2, Math.PI * 0.9));
        m_bottomPID.setConstraints(new Constraints(2.2, Math.PI * 0.9));
    } else {
        m_topPID.setConstraints(new Constraints(2.2, Math.PI * 1.5));
        m_bottomPID.setConstraints(new Constraints(2.2, Math.PI * 1.5));
    }
  }

  public void resetMotionProfile() {
    m_topPID.reset(m_topEncoder.getPosition(), m_topEncoder.getVelocity());
    m_bottomPID.reset(m_bottomEncoder.getPosition(), m_bottomEncoder.getVelocity());
  }

  public double[] calculateAngle(double L1, double L2, double x, double z) {
    double zx = (Math.pow(x,2)+Math.pow(z,2));
    //angle2 --> top arm
    double angle2 = Math.acos((Math.pow(L1,2)+Math.pow(L2,2) - zx)/(2*L1*L2)); //gives angle in radians
    //angle1 --> bottom arm
    double angle1 = (Math.atan2(z,x)+Math.acos((Math.pow(L2,2)-zx-Math.pow(L1,2))/(-2*Math.sqrt(zx)*L1))); // gives angle in radians
    double[] angles = {angle1,angle2};
    return angles;
  }
  public void setBottomJoint(double angle) { // only for testing will switch to bottom arm
    double pidSetpoint = m_topPID.getPIDSetpoint(angle); 
    m_pidControllerTopArm.setReference(pidSetpoint, ControlType.kPosition, 0, calculateFFTop()); 
    m_topArmSetpointPub.accept(Math.toDegrees(angle));
    // double pidSetpoint = m_bottomPID.getPIDSetpoint(angle); 
    // m_pidControllerBottomArm.setReference(pidSetpoint, ControlType.kPosition, 0, calculateFFBottom()); 
    // m_bottomArmSetpointPub.accept(Math.toDegrees(angle));
  }
  public void setTopJoint(double angle) {
    double pidSetpoint = m_topPID.getPIDSetpoint(angle); 
    m_pidControllerTopArm.setReference(pidSetpoint, ControlType.kPosition, 0, calculateFFTop());
    m_topArmSetpointPub.accept(Math.toDegrees(angle));
  }
  public void resetEncoder() {
    m_topArm.getEncoder().setPosition(0);
    // m_bottomArm.getEncoder().setPosition(0);
  }
  private double calculateFFTop() {
    return m_topArmVoltsAtHorizontal.get() * Math.cos(m_topEncoder.getPosition());
  }

  private double calculateFFBottom() {
    return m_bottomArmVoltsAtHorizontal.get() * Math.cos(m_bottomEncoder.getPosition());
  }

  public void setTopArmIdleMode(IdleMode mode) {
    m_topArm.setIdleMode(mode);
  }

  public void setBottomArmIdleMode(IdleMode mode) {
    m_bottomArm.setIdleMode(mode);
  }

  public void testFeedForwardTop(double additionalVoltage) {
    double voltage = additionalVoltage + calculateFFTop();
    m_pidControllerTopArm.setReference(voltage, ControlType.kVoltage);
    m_topArmFFTestingVolts.accept(voltage);
}

  public void testFeedForwardBottom(double additionalVoltage) {
    double voltage = additionalVoltage + calculateFFBottom();
    m_pidControllerBottomArm.setReference(voltage, ControlType.kVoltage);
    m_bottomArmFFTestingVolts.accept(voltage);
  }

  public void stopMotors() {
    m_topArm.stopMotor();
    // m_bottomArm.stopMotor();
}



}
