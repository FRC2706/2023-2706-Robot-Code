// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class armSubsystem extends SubsystemBase {
  DoubleSolenoid breakSolenoidLow;
  DoubleSolenoid breakSolenoidHigh;
  /** Creates a new armSubsystem. */
  public armSubsystem() {
    breakSolenoidLow = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                          PneumaticsModuleType.CTREPCM,
                                          Config.ARMLOW_PNEUMATIC_FORWARD_CHANNEL,
                                          Config.ARMLOW_PNEUMATIC_REVERSE_CHANNEL);
    breakSolenoidHigh = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID,
                                          PneumaticsModuleType.CTREPCM,
                                          Config.ARMHIGH_PNEUMATIC_FORWARD_CHANNEL,
                                          Config.ARMHIGH_PNEUMATIC_REVERSE_CHANNEL);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /*
   * This method will control the Bottom Arm Break
   * @param bBreakOn true: set break mode
   *                 false: remove break mode
   */
  public void controlBottomArmBreak( boolean bBreakOn)
  {
    if (bBreakOn == true)
    {
      //set break on the arm
      breakSolenoidLow.set(Value.kForward);

    }
    else 
    {
      breakSolenoidLow.set(Value.kReverse);
    }
  }
  /*
   * This method will control the Top Arm Break
   * @param bBreakon true: set break mode
   *                 false: remove break mode
   */
  public void controlTopArmBreak( boolean bBreakOn)
  {
    if (bBreakOn == true)
    {
      //set break on the arm
      breakSolenoidHigh.set(Value.kForward);

    }
    else 
    {
      breakSolenoidHigh.set(Value.kReverse);
    }
  }

  public void stopLowBreak() {
    breakSolenoidLow.set(Value.kOff);
  }
  public void stopHighBreak() {
    breakSolenoidHigh.set(Value.kOff);
  }
}
