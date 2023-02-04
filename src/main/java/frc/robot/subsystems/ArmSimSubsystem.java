// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSimSubsystem extends SubsystemBase {
 //**********************************************************/
 // This code was written in referral to Erik's code block
 //**********************************************************/
  // THESES SHOULD BE IN CONFIG
  private static final double m_arm0Reduction = 60;
  private static final double m_arm0Mass = 5.0; // Kilograms
  private static final double m_arm0Length = Units.inchesToMeters(30);
  private static final double m_arm0Noise = 2.0 * Math.PI / 4096;
  private final double encoder0PosConversion = 2 * Math.PI / m_arm0Reduction;

  CANSparkMax m_motor0;
  SparkMaxPIDController m_sparkPid0;
  RelativeEncoder m_encoder0;
  ProfiledPIDController m_pid0;

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower =
          m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90, 5, new Color8Bit(Color.kBlue)));
          
  private final MechanismLigament2d m_arm =
          m_armPivot.append(
              new MechanismLigament2d(
                  "Arm",
                  30,
                  90,
                  6,
                  new Color8Bit(Color.kYellow)));


  // Simulation classes help us simulate what's going on, including gravity.

  // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1),
          m_arm0Reduction,
          SingleJointedArmSim.estimateMOI(m_arm0Length, m_arm0Mass),
          m_arm0Length,
          Units.degreesToRadians(-75),
          Units.degreesToRadians(255),
          m_arm0Mass,
          true,
          VecBuilder.fill(m_arm0Noise) // Add noise with a small std-dev
      );

  
  private static ArmSimSubsystem INSTANCE;
  public static ArmSimSubsystem getInstance() {
      if (INSTANCE == null) {
          INSTANCE = new ArmSimSubsystem();
      }
      return INSTANCE;
  }

  /** Creates a new ArmSimSubsystem. */
  public ArmSimSubsystem() {
      m_motor0 = new CANSparkMax(12, MotorType.kBrushless);

      m_motor0.restoreFactoryDefaults();

      m_sparkPid0 = m_motor0.getPIDController();
      m_encoder0 = m_motor0.getEncoder();

      m_motor0.setInverted(false);
      m_motor0.setSmartCurrentLimit(40);
      m_motor0.setIdleMode(IdleMode.kCoast);

      m_encoder0.setPositionConversionFactor(encoder0PosConversion);
      m_encoder0.setPosition(0);

      m_pid0 = new ProfiledPIDController(
          1, 0, 0, 
          new TrapezoidProfile.Constraints(0, 0));

      // Put Mechanism 2d to SmartDashboard
      SmartDashboard.putData("Arm Sim", m_mech2d);
     
  }

  @Override
  public void periodic() {
      // Update the Mechanism Arm angle based on the simulated arm angle
      m_arm.setAngle(Units.radiansToDegrees(m_encoder0.getPosition()));
  }

  @Override
  public void simulationPeriodic() {
      double motor0AppliedOutput = m_motor0.getAppliedOutput();

      if (DriverStation.isDisabled()) {
          motor0AppliedOutput = 0;
      }

      // In this method, we update our simulation of what our arm is doing
      // First, we set our "inputs" (voltages)
      m_armSim.setInput(motor0AppliedOutput * RobotController.getBatteryVoltage()); 

      // Next, we update it. The standard loop time is 20ms.
      m_armSim.update(0.020);

      // Finally, we set our simulated encoder's readings and simulated battery voltage
      m_encoder0.setPosition(m_armSim.getAngleRads());

      // SimBattery estimates loaded battery voltages
      RoboRioSim.setVInVoltage(
          BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));
  }

  public void setAngle(double angle) {
      double voltage0 = m_pid0.calculate(m_encoder0.getPosition(), angle);

      m_sparkPid0.setReference(voltage0, ControlType.kVoltage);
  }

  public void stopMotors() {
      // Set reference to 0 for arm sim
      m_sparkPid0.setReference(0, ControlType.kVoltage);

      m_motor0.stopMotor();

  }

}
