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
  public enum BLING_COLOUR
  {
    DISABLED,
    PURPLE,
    BLUE,
    RED,
    YELLOW,
    HONEYDEW,
    RAINBOW,
    FIRE,
    RGBFADE,
    WHITESTROBE,
    REDSTROBE,
    YELLOWSTROBE,
    BLUESTROBE,
    PURPLESTROBE
  } 
  public BLING_COLOUR m_blingPatternId;
  public BlingSubsystem bling = BlingSubsystem.getINSTANCE();
  public SetBlingCommand(BLING_COLOUR patternId) {
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
      if (m_blingPatternId != BLING_COLOUR.DISABLED) {
        bling.setBrightness();
      }
      switch( m_blingPatternId )
      {
        case DISABLED:
          bling.setDisabled();
          break;
        case PURPLE:
          bling.setPurple();
          break;
        case BLUE:
          bling.setBlue();
          break;
        case RED:
          bling.setRed();
          break;
        case YELLOW:
          bling.setYellow();
          break;
        case HONEYDEW:
          bling.setHoneydew();
          break;
        case RAINBOW: 
          setRainbow();
          break;
        case FIRE: 
          setFire();
          break;
        case RGBFADE:
          setRgbFade();
          break;
        case WHITESTROBE:
          setWhiteStrobe();
          break;
        case REDSTROBE:
          setRedStrobe();
          break;
        case YELLOWSTROBE:
          setYellowStrobe();
          break;
        case BLUESTROBE:
          setBlueStrobe();
          break;
        case PURPLESTROBE:
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

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}

