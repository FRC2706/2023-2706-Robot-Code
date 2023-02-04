// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BlingSubsystem;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
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
          bling.setHoneydew();
          break;
        case 5: 
          setRainbow();
          break;
        case 6: 
           setFire();
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
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);

    bling.candle.animate(rainbowAnim);

  }

  public void setFire()
  {
    FireAnimation fireAnimation = new FireAnimation(1, 0.5, 64, 0.8, 0.4);

    bling.candle.animate(fireAnimation);
  }

}

