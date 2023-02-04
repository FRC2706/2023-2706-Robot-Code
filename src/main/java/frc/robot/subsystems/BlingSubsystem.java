/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import frc.robot.config.Config;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class BlingSubsystem extends SubsystemBase {

  public CANdle candle; 
  private static BlingSubsystem INSTANCE = null;
  /**
   * Creates a new Bling.
   */
  private BlingSubsystem() {
    if ( Config.CANID.CANDLE != -1 )
    {
      candle = new CANdle( Config.CANID.CANDLE );

      CANdleConfiguration config = new CANdleConfiguration();
      config.stripType = LEDStripType.RGB; // set the strip type to RGB
      config.brightnessScalar = 0.5; // dim the LEDs to half brightness

      candle.configAllSettings(config);
    }
    else
    {
      candle = null;
    }    

  }

  public static BlingSubsystem getINSTANCE() {
    if ( Config.CANID.CANDLE == -1 )
    {
      INSTANCE = null;
    }
    else if ( INSTANCE == null )
    {
      INSTANCE = new BlingSubsystem();
    }

    return INSTANCE;
  }

  public void setPurple()
  {
    candle.setLEDs(138, 43, 226);
  }

  public void setBlue()
  {
    candle.setLEDs(0, 0, 255);
  }

  public void setRed()
  {
    candle.setLEDs(255, 0, 0);
  }

  public void setHoneydew()
  {
    candle.setLEDs(240, 255, 240);
  }

  public void setRainbow()
  {
    // create a rainbow animation:
    // - max brightness
    // - half speed
    // - 64 LEDs
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.9, 64);

    candle.animate(rainbowAnim);

  }

  public void setFire()
  {
    FireAnimation fireAnimation = new FireAnimation(0.5, 0.7, 6, 0.7, 0.5);
    candle.animate(fireAnimation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      
  }
}