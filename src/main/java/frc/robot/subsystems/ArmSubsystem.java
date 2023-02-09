// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class ArmSubsystem extends SubsystemBase {
  DoubleSolenoid brakeSolenoidLow;
  DoubleSolenoid brakeSolenoidHigh;
  /** Creates a new armSubsystem. */
  public ArmSubsystem() {
    brakeSolenoidLow = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                          PneumaticsModuleType.CTREPCM,
                                          Config.ARMLOW_PNEUMATIC_FORWARD_CHANNEL,
                                          Config.ARMLOW_PNEUMATIC_REVERSE_CHANNEL);
    brakeSolenoidHigh = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                          PneumaticsModuleType.CTREPCM,
                                          Config.ARMHIGH_PNEUMATIC_FORWARD_CHANNEL,
                                          Config.ARMHIGH_PNEUMATIC_REVERSE_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /*
   * This method will control the Bottom Arm Brake
   * @param bBrakeOn true: set brake mode
   *                 false: remove brake mode
   */
  public void controlBottomArmBrake( boolean bBrakeOn)
  {
    if (bBrakeOn == true)
    {
      //set brake on the arm
      brakeSolenoidLow.set(Value.kForward);

    }
    else 
    {
      brakeSolenoidLow.set(Value.kReverse);
    }
  }
  /*
   * This method will control the Top Arm Brake
   * @param bBrakeon true: set brake mode
   *                 false: remove brake mode
   */
  public void controlTopArmBrake( boolean bBrakeOn)
  {
    if (bBrakeOn == true)
    {
      //set brake on the arm
      brakeSolenoidHigh.set(Value.kForward);

    }
    else 
    {
      brakeSolenoidHigh.set(Value.kReverse);
    }
  }

}
