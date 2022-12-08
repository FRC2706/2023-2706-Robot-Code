// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.config.Config;
import frc.robot.config.FluidConstant;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubSystem extends SubsystemBase {
  /** Creates a new IndexerSubSystem. */
  private CANSparkMax m_motor;
  
  private static IndexerSubSystem instance;

  public static IndexerSubSystem getInstance() {
    if (instance==null){
      instance = new IndexerSubSystem();
    }
    return instance;
  }
  
  public void setSpeed(double speed){
    m_motor.set(0.3);
  }

  public void stopMotors(){
    m_motor.stopMotor();
  }

   public static FluidConstant<Double> INDEXER_RPM = new FluidConstant<>
  ("Indexer_PRM",700.).registerToTable(Config.constantsTable);

  private CANSparkMax m_indexer;
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double targetRPM = INDEXER_RPM.getValue();
  public double kMaxOutput = 1;
  public double kMinOutput = -1;
  public double currentPosition = 0;
  public boolean m_bGoodSensors = false;
  private static final IndexerSubSystem INDEXER_SUB_SYSTEM = new IndexerSubSystem();

  private IndexerSubSystem() {

    
      m_indexer = new CANSparkMax(Config.CANID.INDEXER, MotorType.kBrushless);
  
      if ( m_indexer != null )
      {      
        m_bGoodSensors = true;
    
        // Factory Default to prevent unexpected behaviour
        m_indexer.restoreFactoryDefaults();
        m_indexer.setInverted(false);
      }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
