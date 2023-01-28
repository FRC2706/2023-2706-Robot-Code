//set three settings for claw open, cube and cone(In Progress)
//think of better name then 1 and 2
//
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;
public class IntakeSubsytem extends SubsystemBase {
  DoubleSolenoid doubleSolenoid1;
  DoubleSolenoid doubleSolenoid2;
  private static final IntakeSubsystem Claw_PNEUMATIC_SUBSYSTEM = new IntakeSubsystem();

  /** Creates a new ClawPneumaticSubsystem. */
  public IntakeSubsytem() 
  {
    
    if (Config.CTRE_INTAKE_PCM_CAN_ID == -1 
    || Config.INTAKE1_PNEUMATIC_FORWARD_CHANNEL == -1 
    || Config.INTAKE1_PNEUMATIC_REVERSE_CHANNEL == -1)
    {
      doubleSolenoid1 = null;
    }
    else
    {
    doubleSolenoid1 = new DoubleSolenoid(Config.CTRE_INTAKE_PCM_CAN_ID, 
                                               PneumaticsModuleType.CTREPCM,
                                               Config.INTAKE1_PNEUMATIC_FORWARD_CHANNEL,
                                               Config.INTAKE1_PNEUMATIC_REVERSE_CHANNEL);
    }

    if(Config.CTRE_INTAKE_PCM_CAN_ID == -1 
    || Config.INTAKE2_PNEUMATIC_CHANNEL_1 == -1 
    || Config.INTAKE2_PNEUMATIC_CHANNEL_2 == -1)
    {
      doubleSolenoid2 = null;
    }
    else
    {
      doubleSolenoid2 = new DoubleSolenoid(Config.CTRE_INTAKE_PCM_CAN_ID, 
                                              PneumaticsModuleType.CTREPCM,
                                              Config.INTAKE2_PNEUMATIC_CHANNEL_1,
                                              Config.INTAKE2_PNEUMATIC_CHANNEL_2);
    }
  }

  public boolean isActive()
  {
    if(doubleSolenoid1 == null || doubleSolenoid2 == null)
    {
      return false;
    } 
    else
    {
      return true;
    }
  }
   /**
   * Returns the singleton instance for the ClawSubsystem
   */
  public static IntakeSubsystem getInstance()
  {
    if (Claw_PNEUMATIC_SUBSYSTEM.isActive())
      return Claw_PNEUMATIC_SUBSYSTEM;
    else
      return null;
  }

  public void lowPressure()
  {
    doubleSolenoid1.set(Value.kForward);
    doubleSolenoid2.set(Value.kReverse);

  }

  public void highPressure()
  {
    doubleSolenoid1.set(Value.kForward);
    doubleSolenoid2.set(Value.kForward);
  }

  public void noPressure()
  {
    doubleSolenoid1.set(Value.kReverse);
    doubleSolenoid2.set(Value.kReverse);
  }

  public void stop()
  {
    doubleSolenoid1.set(Value.kOff);
    doubleSolenoid2.set(Value.kOff);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}