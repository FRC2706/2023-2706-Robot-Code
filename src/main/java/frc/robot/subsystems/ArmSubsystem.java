// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.ErrorCheck.errREV;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ProfileExternalPIDController;
import frc.robot.SubsystemChecker;
import frc.robot.SubsystemChecker.SubsystemType;
import frc.robot.config.ArmConfig;
import frc.robot.config.Config;


public class ArmSubsystem extends SubsystemBase {

  private ArmDisplay armDisplay;

  private static final MotorType motorType = MotorType.kBrushless;
  private static final SparkMaxAbsoluteEncoder.Type encAbsType = SparkMaxAbsoluteEncoder.Type.kDutyCycle;
  
  // public CANCoder m_absoluteTopArmEncoder;
  // public CANCoder m_absoluteBottomArmEncoder;
  private static ArmSubsystem instance = null;
  public final CANSparkMax m_topArm;
  public final CANSparkMax m_bottomArm;
  public SparkMaxPIDController m_pidControllerTopArm;
  public SparkMaxPIDController m_pidControllerBottomArm;
  public ProfileExternalPIDController m_topPID;
  public ProfileExternalPIDController m_bottomPID;
  private RelativeEncoder m_bottomEncoder;
  private RelativeEncoder m_topEncoder;

  private double m_topArmEncoderOffset;
  private double m_bottomArmEncoderOffset;

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
  private DoubleEntry m_topArmOffset;
  private DoublePublisher m_topAbsoluteEncoder;

  // network table entries for bottom arm
  private DoubleEntry m_bottomArmPSubs;
  private DoubleEntry m_bottomArmISubs;
  private DoubleEntry m_bottomArmDSubs;
  private DoubleEntry m_bottomArmIzSubs;
  private DoubleEntry m_bottomArmFFSubs;
  private DoublePublisher m_bottomArmPosPub;
  private DoublePublisher m_bottomArmSetpointPub;   
  private DoublePublisher m_bottomArmVelPub;
  private DoubleEntry m_bottomArmMomentToVoltage;
  private DoublePublisher m_bottomArmFFTestingVolts;
  private DoubleEntry m_bottomArmOffset;
  private DoublePublisher m_bottomAbsoluteEncoder;

  // for bottom arm ff
  private DoubleSubscriber momentToVoltageConversion;
  private double m_bottomVoltageConversion;
  private boolean m_hasCone = false;

  // for arm pneumatic brakes
  DoubleSolenoid brakeSolenoidLow;
  DoubleSolenoid brakeSolenoidHigh;

  // duty cycle encoder
  private DutyCycleEncoder m_topDutyCycleEncoder;
  private DutyCycleEncoder m_bottomDutyCycleEncoder;

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
    m_topArm.restoreFactoryDefaults();
    m_bottomArm.restoreFactoryDefaults();
    // m_topArm.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT);
    m_bottomArm.setSmartCurrentLimit(ArmConfig.CURRENT_LIMIT);
    m_topArm.setInverted(ArmConfig.TOP_SET_INVERTED);
    m_bottomArm.setInverted(ArmConfig.BOTTOM_SET_INVERTED);
    m_topArm.setIdleMode(IdleMode.kBrake);
    m_bottomArm.setIdleMode(IdleMode.kBrake);
    m_topArm.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.top_arm_forward_limit);
    m_topArm.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.top_arm_reverse_limit);
    m_bottomArm.setSoftLimit(SoftLimitDirection.kForward, ArmConfig.bottom_arm_forward_limit);
    m_bottomArm.setSoftLimit(SoftLimitDirection.kReverse, ArmConfig.bottom_arm_reverse_limit);
    m_topArm.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.TOP_SOFT_LIMIT_ENABLE);
    m_topArm.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.TOP_SOFT_LIMIT_ENABLE);
    m_bottomArm.enableSoftLimit(SoftLimitDirection.kForward, ArmConfig.BOTTOM_SOFT_LIMIT_ENABLE);
    m_bottomArm.enableSoftLimit(SoftLimitDirection.kReverse, ArmConfig.BOTTOM_SOFT_LIMIT_ENABLE);

    m_topDutyCycleEncoder = new DutyCycleEncoder(ArmConfig.top_duty_cycle_channel);
    m_topDutyCycleEncoder.setDistancePerRotation(360);
    m_bottomDutyCycleEncoder = new DutyCycleEncoder(ArmConfig.bottom_duty_cycle_channel);
    m_bottomDutyCycleEncoder.setDistancePerRotation(360);
    
    CANCoderConfiguration config = new CANCoderConfiguration();
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    config.magnetOffsetDegrees = 0;
    config.sensorDirection = true;

    armDisplay = new ArmDisplay(ArmConfig.L1, ArmConfig.L2);

    // m_absoluteTopArmEncoder = new CANCoder(Config.CANID.TOP_CANCODER_CAN_ID);
    // m_absoluteBottomArmEncoder = new CANCoder(Config.CANID.BOTTOM_CANCODER_CAN_ID);

    brakeSolenoidLow = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                          PneumaticsModuleType.CTREPCM,
                                          Config.ARMLOW_PNEUMATIC_FORWARD_CHANNEL,
                                          Config.ARMLOW_PNEUMATIC_REVERSE_CHANNEL);

    brakeSolenoidHigh = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                          PneumaticsModuleType.CTREPCM,
                                          Config.ARMHIGH_PNEUMATIC_FORWARD_CHANNEL,
                                          Config.ARMHIGH_PNEUMATIC_REVERSE_CHANNEL);

    // m_absoluteTopArmEncoder.configAllSettings(config);
    // m_absoluteBottomArmEncoder.configAllSettings(config);

    m_pidControllerTopArm = m_topArm.getPIDController();
    m_pidControllerBottomArm = m_bottomArm.getPIDController();
    m_topPID = new ProfileExternalPIDController(new Constraints(ArmConfig.TOP_MAX_VEL, ArmConfig.TOP_MAX_ACCEL));
    m_bottomPID = new ProfileExternalPIDController(new Constraints(ArmConfig.BOTTOM_MAX_VEL, ArmConfig.BOTTOM_MAX_ACCEL));

    m_bottomEncoder = m_bottomArm.getEncoder();
    m_topEncoder = m_topArm.getEncoder();
    
    m_bottomEncoder.setPositionConversionFactor(ArmConfig.bottomArmPositionConversionFactor);
    m_topEncoder.setPositionConversionFactor(ArmConfig.topArmPositionConversionFactor);

    m_topEncoder.setVelocityConversionFactor(ArmConfig.topArmVelocityConversionFactor);
    m_bottomEncoder.setVelocityConversionFactor(ArmConfig.bottomArmVelocityConversionFactor);

    NetworkTable topArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableTop);
    m_topArmPSubs = topArmTuningTable.getDoubleTopic("P").getEntry(ArmConfig.top_arm_kP);
    m_topArmISubs = topArmTuningTable.getDoubleTopic("I").getEntry(ArmConfig.top_arm_kI);
    m_topArmDSubs = topArmTuningTable.getDoubleTopic("D").getEntry(ArmConfig.top_arm_kD);
    m_topArmIzSubs = topArmTuningTable.getDoubleTopic("IZone").getEntry(ArmConfig.top_arm_kIz);
    m_topArmFFSubs = topArmTuningTable.getDoubleTopic("FF").getEntry(ArmConfig.top_arm_kFF);
    m_topArmOffset = topArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.top_arm_offset);
    
    NetworkTable bottomArmTuningTable = NetworkTableInstance.getDefault().getTable(m_tuningTableBottom);
    m_bottomArmPSubs = bottomArmTuningTable.getDoubleTopic("P").getEntry(ArmConfig.bottom_arm_kP);
    m_bottomArmISubs = bottomArmTuningTable.getDoubleTopic("I").getEntry(ArmConfig.bottom_arm_kI);
    m_bottomArmDSubs = bottomArmTuningTable.getDoubleTopic("D").getEntry(ArmConfig.bottom_arm_kD);
    m_bottomArmIzSubs = bottomArmTuningTable.getDoubleTopic("IZone").getEntry(ArmConfig.bottom_arm_kIz);
    m_bottomArmFFSubs = bottomArmTuningTable.getDoubleTopic("FF").getEntry(ArmConfig.bottom_arm_kFF);
    m_bottomArmOffset = bottomArmTuningTable.getDoubleTopic("Offset").getEntry(ArmConfig.bottom_arm_offset);
    momentToVoltageConversion = bottomArmTuningTable.getDoubleTopic("VoltageConversion").subscribe(m_bottomVoltageConversion);

    // if (m_topArmPSubs.getAtomic().timestamp == 0) {
        m_topArmFFSubs.accept(ArmConfig.top_arm_kFF);
        m_topArmPSubs.accept(ArmConfig.top_arm_kP);
        m_topArmISubs.accept(ArmConfig.top_arm_kI);
        m_topArmDSubs.accept(ArmConfig.top_arm_kD);
        m_topArmIzSubs.accept(ArmConfig.top_arm_kIz);
        m_topArmOffset.accept(ArmConfig.top_arm_offset);
    // }

    // if (m_bottomArmPSubs.getAtomic().timestamp == 0) {
      m_bottomArmFFSubs.accept(ArmConfig.bottom_arm_kFF);
      m_bottomArmPSubs.accept(ArmConfig.bottom_arm_kP);
      m_bottomArmISubs.accept(ArmConfig.bottom_arm_kI);
      m_bottomArmDSubs.accept(ArmConfig.bottom_arm_kD);
      m_bottomArmIzSubs.accept(ArmConfig.bottom_arm_kIz);
      m_bottomArmOffset.accept(ArmConfig.bottom_arm_offset);
  // }

    NetworkTable topArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTableTop);
    m_topArmPosPub = topArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_topArmSetpointPub = topArmDataTable.getDoubleTopic("SetpointAngle").publish(PubSubOption.periodic(0.02));
    m_topArmVelPub = topArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));
    m_topArmVoltsAtHorizontal = topArmDataTable.getDoubleTopic("VoltsAtHorizontal").getEntry(0);
    m_topArmVoltsAtHorizontal.accept(ArmConfig.TOP_HORIZONTAL_VOLTAGE);
    m_topAbsoluteEncoder = topArmDataTable.getDoubleTopic("Absolute Encoder").publish(PubSubOption.periodic(0.02));

    m_topArmFFTestingVolts = topArmDataTable.getDoubleTopic("VoltageSetInFFTesting").publish();

    NetworkTable bottomArmDataTable = NetworkTableInstance.getDefault().getTable(m_dataTableBottom);
    m_bottomArmPosPub = bottomArmDataTable.getDoubleTopic("MeasuredAngle").publish(PubSubOption.periodic(0.02));
    m_bottomArmSetpointPub = bottomArmDataTable.getDoubleTopic("SetpointAngle").publish(PubSubOption.periodic(0.02));
    m_bottomArmVelPub = bottomArmDataTable.getDoubleTopic("Vel").publish(PubSubOption.periodic(0.02));
    m_bottomArmMomentToVoltage = bottomArmDataTable.getDoubleTopic("MomentVoltage").getEntry(0);
    m_bottomArmMomentToVoltage.accept(ArmConfig.BOTTOM_MOMENT_TO_VOLTAGE);
    m_bottomAbsoluteEncoder = bottomArmDataTable.getDoubleTopic("Absolute Encoder").publish(PubSubOption.periodic(0.02));

    m_bottomArmFFTestingVolts = bottomArmDataTable.getDoubleTopic("VoltageSetInFFTesting").publish();

    updatePIDSettings();
    updateFromAbsoluteTop();
    updateFromAbsoluteBottom();
  }

  public void updatePIDSettings() {
    // setting PID constants for top spark max
    m_pidControllerTopArm.setFF(m_topArmFFSubs.get());
    m_pidControllerTopArm.setP(m_topArmPSubs.get());
    m_pidControllerTopArm.setI(m_topArmISubs.get());
    m_pidControllerTopArm.setD(m_topArmDSubs.get());
    m_pidControllerTopArm.setIZone(m_topArmIzSubs.get()); 
    m_pidControllerTopArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output);

    // setting PID constants for top spark max
    m_pidControllerTopArm.setFF(ArmConfig.top_arm_kFF2, 1);
    m_pidControllerTopArm.setP(ArmConfig.top_arm_kP2, 1);
    m_pidControllerTopArm.setI(ArmConfig.top_arm_kI2, 1);
    m_pidControllerTopArm.setD(ArmConfig.top_arm_kD2, 1);
    m_pidControllerTopArm.setIZone(ArmConfig.top_arm_kIz2, 1);
    m_pidControllerTopArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output);
    
    // setting PID constants for bottom spark max
    m_pidControllerBottomArm.setFF(m_bottomArmFFSubs.get());
    m_pidControllerBottomArm.setP(m_bottomArmPSubs.get());
    m_pidControllerBottomArm.setI(m_bottomArmISubs.get());
    m_pidControllerBottomArm.setD(m_bottomArmDSubs.get());
    m_pidControllerBottomArm.setIZone(m_bottomArmIzSubs.get()); 
    m_pidControllerBottomArm.setOutputRange(ArmConfig.min_output, ArmConfig.max_output);
}
  PowerDistribution pdp = new PowerDistribution(); 
  @Override
  public void periodic() {
    double topPosition = m_topEncoder.getPosition();
    double bottomPosition = m_bottomEncoder.getPosition();

    m_topArmPosPub.accept(Math.toDegrees(topPosition));
    m_topArmVelPub.accept(m_topEncoder.getVelocity());
    m_bottomArmPosPub.accept(Math.toDegrees(bottomPosition));
    m_bottomArmVelPub.accept(m_bottomEncoder.getVelocity());
    m_topAbsoluteEncoder.accept(Math.toDegrees(getAbsoluteTop()));
    m_bottomAbsoluteEncoder.accept(Math.toDegrees(getAbsoluteBottom()));

    armDisplay.updateMeasurementDisplay(bottomPosition, topPosition);


    // pdp.getCurrent(12);
    SmartDashboard.putNumber("BotArmCurrent", pdp.getCurrent(12));
    SmartDashboard.putNumber("TopArmCurrent", pdp.getCurrent(15));
    //12, Lower 
    // 15, Upper



  }

  public void resetMotionProfile() {
    m_topPID.reset(m_topEncoder.getPosition(), m_topEncoder.getVelocity());
    m_bottomPID.reset(m_bottomEncoder.getPosition(), m_bottomEncoder.getVelocity());
  }

  public double[] inverseKinematics(double L1, double L2, double x, double z) {
    double zx = (Math.pow(x,2)+Math.pow(z,2));
    //angle2 --> top arm
    double angle2 = Math.acos((Math.pow(L1,2)+Math.pow(L2,2) - zx)/(2*L1*L2)); //gives angle in radians
    //angle1 --> bottom arm
    double angle1 = (Math.atan2(z,x)+Math.acos((Math.pow(L2,2)-zx-Math.pow(L1,2))/(-2*Math.sqrt(zx)*L1))); // gives angle in radians
    double[] angles = {angle1,angle2};
    return angles;
  }

  public void setBottomJoint(double angle_bottom, double angle_top) { 
    setBottomJoint(angle_bottom, angle_top, 0);
  }

  public void setBottomJoint(double angle_bottom, double angle_top, double vel) { 
    if (angle_top > Math.toRadians(20) && getTopPosition() < Math.toRadians(20)) {
      angle_bottom = Math.toRadians(95);
    }
    double pidSetpoint = m_bottomPID.getPIDSetpoint(new TrapezoidProfile.State(angle_bottom, vel)); 
    m_pidControllerBottomArm.setReference(pidSetpoint, ControlType.kPosition, 0, calculateFFBottom(m_bottomEncoder.getPosition(), m_topEncoder.getPosition(), m_hasCone)); 
    m_bottomArmSetpointPub.accept(Math.toDegrees(angle_bottom));
  }
  public void setTopJoint(double angle) {
    setTopJoint(angle, 0);
  }

  public void setTopJoint(double angle, double vel) {
    double topVel = getTopVel();
    // if (angle < Math.toRadians(25) && getTopVel() < -1.5) {
    //   angle = Math.toRadians(24);
    // }

    double pidSetpoint = m_topPID.getPIDSetpoint(new TrapezoidProfile.State(angle, vel)); 

    double slowDownVoltage = 0;
    if (topVel < Math.toRadians(-160)) {
      slowDownVoltage += 0.6;
    }

    double velRange = Math.toRadians(-40);
    if (topVel < velRange) {
      slowDownVoltage += (topVel - velRange) * -1.2;
    }

    if (getTopPosition() < 40 && topVel < Math.toRadians(-60)) {
      slowDownVoltage += getTopVel() * -0.9;
    }

    // Overshoot then prevent it from going below the setpoint
    if (angle > Math.toRadians(50) && getTopPosition() > angle && getTopPosition() - angle < Math.toRadians(10) && getTopVel() < 0) {
      if (m_hasCone) {
        slowDownVoltage += 0.6;
      } else {
        slowDownVoltage += 0.2;
      }
    }

    // Stop the middle cone from overshooting too much
    if (angle > Math.toRadians(35) && getTopVel() > 1.5 && getTopPosition() < angle && angle - getTopPosition() < Math.toRadians(25)) {
      slowDownVoltage -= 1;
    }

    int slot = 0;
    if (m_hasCone) {
      slot = 1;
    } else {
      slot = 0;
    }
    m_pidControllerTopArm.setReference(pidSetpoint, ControlType.kPosition, slot, calculateFFTop(m_hasCone, angle) + slowDownVoltage);//+ m_topSimpleFF.calculate(m_topPID.getSetpoint().velocity, acceleration));
    m_topArmSetpointPub.accept(Math.toDegrees(angle));
  }

  public void resetEncoder(double bottom_position, double top_position) {
    m_topArm.getEncoder().setPosition(top_position);
    m_bottomArm.getEncoder().setPosition(bottom_position);
  }
 
  private double calculateFFTop(boolean haveCone, double topSetpoint) {
    double enc2AtHorizontal = getTopPosition() - (Math.PI - getBottomPosition());
    double voltsAtHorizontal;
    if (haveCone) {
        voltsAtHorizontal = ArmConfig.TOP_HORIZONTAL_VOLTAGE_CONE;
    }
    else {
      voltsAtHorizontal = m_topArmVoltsAtHorizontal.get();
    }
    // System.out.println(voltsAtHorizontal * Math.cos(enc2AtHorizontal));
    return voltsAtHorizontal * Math.cos(enc2AtHorizontal);
  }

  private double calculateFFBottom(double encoder1Rad, double encoder2Rad, boolean haveCone) {
    double enc2AtHorizontal = encoder2Rad - (Math.PI - encoder1Rad);
    double bottomArmMoment = ArmConfig.BOTTOM_ARM_FORCE * (ArmConfig.LENGTH_BOTTOM_ARM_TO_COG*Math.cos(encoder1Rad));
    double topArmMoment = ArmConfig.TOP_ARM_FORCE * (ArmConfig.L1*Math.cos(encoder1Rad) + ArmConfig.LENGTH_TOP_ARM_TO_COG*Math.cos(enc2AtHorizontal));
    if (haveCone == false) {
        return (bottomArmMoment + topArmMoment) * m_bottomArmMomentToVoltage.get();
    } 
    else {
        double coneMoment = ArmConfig.CONE_ARM_FORCE * (ArmConfig.L1*Math.cos(encoder1Rad) + ArmConfig.L2*Math.cos(enc2AtHorizontal));
        return (bottomArmMoment + topArmMoment + coneMoment) * m_bottomArmMomentToVoltage.get();
    }
  }

  public void setTopArmIdleMode(IdleMode mode) {
    m_topArm.setIdleMode(mode);
  }

  public void setBottomArmIdleMode(IdleMode mode) {
    m_bottomArm.setIdleMode(mode);
  }

  public void testFeedForwardTop(double additionalVoltage) {
    double voltage = additionalVoltage + calculateFFTop(m_hasCone, 0);
    m_pidControllerTopArm.setReference(voltage, ControlType.kVoltage);
    m_topArmFFTestingVolts.accept(voltage);
}

  public void testFeedForwardBottom(double additionalVoltage) {
    double voltage = additionalVoltage + calculateFFBottom(m_bottomEncoder.getPosition(), m_topEncoder.getPosition(), m_hasCone);
    m_pidControllerBottomArm.setReference(voltage, ControlType.kVoltage);
    m_bottomArmFFTestingVolts.accept(voltage);
  }

  public void stopMotors() {
    m_topArm.stopMotor();
    m_bottomArm.stopMotor();
}

  public double getAbsoluteTop() {
    return Math.toRadians(m_topDutyCycleEncoder.getAbsolutePosition() * 360 + m_topArmOffset.get());
  }

  public double getAbsoluteBottom() {
    return Math.toRadians(m_bottomDutyCycleEncoder.getAbsolutePosition() * -360 + m_bottomArmOffset.get());
  }

  public void updateFromAbsoluteTop() {
    errREV(m_topEncoder.setPosition(getAbsoluteTop()));
  }

  public void updateFromAbsoluteBottom() {
    errREV(m_bottomEncoder.setPosition(getAbsoluteBottom()));
  }

  public void controlBottomArmBrake( boolean bBrakeOn) {
    if (bBrakeOn == true) {
      //set brake on the arm
      brakeSolenoidLow.set(Value.kForward);
    }
    else {
      brakeSolenoidLow.set(Value.kReverse);
    }
  }
  /*
   * This method will control the Top Arm Brake
   * @param bBrakeon true: set brake mode
   *                 false: remove brake mode
   */
  public void controlTopArmBrake( boolean bBrakeOn) {
    if (bBrakeOn == true) {
      //set brake on the arm
      brakeSolenoidHigh.set(Value.kForward);
    }
    else {
      brakeSolenoidHigh.set(Value.kReverse);
    }
  }

  public void updateSetpointDisplay(double setpoint1, double setpoint2) {
    armDisplay.updateSetpointDisplay(setpoint1, setpoint2);
  }

  public double getTopPosition() {
    return m_topEncoder.getPosition();
  }

  public double getBottomPosition() {
    return m_bottomEncoder.getPosition();
  }

  public double getTopVel() {
    return m_topEncoder.getVelocity();
  }

  public double getBottomVel() {
    return m_bottomEncoder.getVelocity();
  }

  public void setHasCone(boolean hasCone) {
    if (hasCone) {
      // m_topPID.setConstraints(new Constraints(ArmConfig.TOP_CONE_MAX_VEL, ArmConfig.TOP_CONE_MAX_ACCEL));
      m_bottomPID.setConstraints(new Constraints(ArmConfig.BOTTOM_CONE_MAX_VEL, ArmConfig.BOTTOM_CONE_MAX_ACCEL));
    }
    else {
      m_topPID.setConstraints(new Constraints(ArmConfig.TOP_MAX_VEL, ArmConfig.TOP_MAX_ACCEL));
      m_bottomPID.setConstraints(new Constraints(ArmConfig.BOTTOM_MAX_VEL, ArmConfig.BOTTOM_MAX_ACCEL));
    }
    m_hasCone = hasCone;
  }

  /**
   * Checks if the Neo encoder is synced with the Absolute Encoder
   * 
   * @return Whether the encoders are synced or not
   */
  public boolean areEncodersSynced() {
    return Math.abs(getAbsoluteBottom() - getBottomPosition()) < ArmConfig.ENCODER_SYNCING_TOLERANCE &&
           Math.abs(getAbsoluteTop() - getTopPosition()) < ArmConfig.ENCODER_SYNCING_TOLERANCE;

  }

  public void setTopConstraints(double maxVelocity, double maxAccel) {
    m_topPID.setConstraints(new Constraints(maxVelocity, maxAccel));
  }

  public void setTopVoltage(double topVoltage) {
    m_topArm.set(topVoltage);
  }
}
