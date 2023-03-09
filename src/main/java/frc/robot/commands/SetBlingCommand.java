// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BlingSubsystem;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import frc.robot.commands.SetBlingCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class SetBlingCommand extends InstantCommand {
  public int m_blingPatternId;
  public BlingSubsystem bling = BlingSubsystem.getINSTANCE();

  public SetBlingCommand(int patternId) {
    m_blingPatternId = patternId;

    // Use addRequirements() here to declare subsystem dependencies.
    if( bling != null )
      addRequirements(bling);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (bling != null)
    {
      if (m_blingPatternId!= 0) {
        bling.setBrightness();
      }
      switch( m_blingPatternId )
      {
        case 0:
          bling.setDisabled();
          break;
        case 1:
          bling.setPurple();
          break;
        case 2:
          bling.setBlue();
          break;
        case 3:
          bling.setRed();
          break;
        case 4:
          bling.setYellow();
          break;
        case 5:
          bling.setHoneydew();
          break;
        case 6: 
          setRainbow();
          break;
        case 7: 
          setFire();
          break;
        case 8:
          setRgbFade();
          break;
        case 9:
          setWhiteStrobe();
          break;
        case 10:
          setRedStrobe();
          break;
        case 11:
          setYellowStrobe();
          break;
        case 12:
          setBlueStrobe();
          break;
        case 13:
          setPurpleStrobe();
          break;
        default:
          break;
      }
    }

    }
  
  public void setRainbow() {
    // create a rainbow animation:
    // - max brightness
    // - half speed
    // - 64 LEDs
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.01, 64);

    bling.candle.animate(rainbowAnim);

  }

  public void setFire()
  {
    FireAnimation fireAnimation = new FireAnimation(1, 0.000001, 64, 0.8, 0.4);

    bling.candle.animate(fireAnimation);
  }

  public void setWhiteStrobe()
  {
    StrobeAnimation strobeAnimation = new StrobeAnimation(255, 255, 255, 255, 0.8, 64);

    bling.candle.animate(strobeAnimation);
  }

  public void setPurpleStrobe()
  {
    StrobeAnimation strobeAnimation = new StrobeAnimation(138, 43, 226, 127, 0.001, 64); //TODO: test all of the rgbw bling values

    bling.candle.animate(strobeAnimation);
  }

  public void setRedStrobe()
  {
    StrobeAnimation strobeAnimation = new StrobeAnimation(255, 0, 0, 127, 0.001, 64);

    bling.candle.animate(strobeAnimation);
  }

  public void setBlueStrobe()
  {
    StrobeAnimation strobeAnimation = new StrobeAnimation(0, 0, 255, 127, 0.001, 64);

    bling.candle.animate(strobeAnimation);
  }

  public void setYellowStrobe()
  {
    StrobeAnimation strobeAnimation = new StrobeAnimation(255, 255, 0, 127, 0.001, 64);

    bling.candle.animate(strobeAnimation);
  }

  public void setRgbFade()
  {
    RgbFadeAnimation rgbFadeAnimation = new RgbFadeAnimation(0.7, 0.1, 64);

    bling.candle.animate(rgbFadeAnimation);
  }

}

