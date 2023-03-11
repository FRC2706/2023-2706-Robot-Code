// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class AnalogSelectorSubsystem extends SubsystemBase {

  private static final double MODE_ZERO_LOW_VOLTAGE = 0;
  private static final double MODE_ZERO_HIGH_VOLTAGE = 2.4;

  private static final double MODE_ONE_LOW_VOLTAGE = 2.4;
  private static final double MODE_ONE_HIGH_VOLTAGE = 2.8;

  private static final double MODE_TWO_LOW_VOLTAGE = 2.8;
  private static final double MODE_TWO_HIGH_VOLTAGE = 3.1;

  private static final double MODE_THREE_LOW_VOLTAGE = 3.1;
  private static final double MODE_THREE_HIGH_VOLTAGE = 3.4;

  private static final double MODE_FOUR_LOW_VOLTAGE = 3.4;
  private static final double MODE_FOUR_HIGH_VOLTAGE = 3.7;

  private static final double MODE_FIVE_LOW_VOLTAGE = 3.7;
  private static final double MODE_FIVE_HIGH_VOLTAGE = 3.9;

  private static final double MODE_SIX_LOW_VOLTAGE = 3.9;
  private static final double MODE_SIX_HIGH_VOLTAGE = 4.05;

  private static final double MODE_SEVEN_LOW_VOLTAGE = 4.05;
  private static final double MODE_SEVEN_HIGH_VOLTAGE = 4.15;

  private static final double MODE_EIGHT_LOW_VOLTAGE = 4.15;
  private static final double MODE_EIGHT_HIGH_VOLTAGE = 4.24;
    
  private static final double MODE_NINE_LOW_VOLTAGE = 4.24;
  private static final double MODE_NINE_HIGH_VOLTAGE = 4.34;

  private static final double MODE_TEN_LOW_VOLTAGE = 4.34;
  private static final double MODE_TEN_HIGH_VOLTAGE = 4.4;

  private static final double MODE_ELEVEN_LOW_VOLTAGE = 4.4;
  private static final double MODE_ELEVEN_HIGH_VOLTAGE = 4.49;

  private static final double MODE_TWELVE_LOW_VOLTAGE = 4.49;
  private static final double MODE_TWELVE_HIGH_VOLTAGE = 5;

  private static final Range[] VOLTAGE_RANGES = {
      new Range(MODE_ZERO_LOW_VOLTAGE, MODE_ZERO_HIGH_VOLTAGE),
      new Range(MODE_ONE_LOW_VOLTAGE, MODE_ONE_HIGH_VOLTAGE), 
      new Range(MODE_TWO_LOW_VOLTAGE, MODE_TWO_HIGH_VOLTAGE),
      new Range(MODE_THREE_LOW_VOLTAGE, MODE_THREE_HIGH_VOLTAGE), 
      new Range(MODE_FOUR_LOW_VOLTAGE, MODE_FOUR_HIGH_VOLTAGE), 
      new Range(MODE_FIVE_LOW_VOLTAGE, MODE_FIVE_HIGH_VOLTAGE),
      new Range(MODE_SIX_LOW_VOLTAGE, MODE_SIX_HIGH_VOLTAGE), 
      new Range(MODE_SEVEN_LOW_VOLTAGE, MODE_SEVEN_HIGH_VOLTAGE), 
      new Range(MODE_EIGHT_LOW_VOLTAGE, MODE_EIGHT_HIGH_VOLTAGE),
      new Range(MODE_NINE_LOW_VOLTAGE, MODE_NINE_HIGH_VOLTAGE), 
      new Range(MODE_TEN_LOW_VOLTAGE, MODE_TEN_HIGH_VOLTAGE), 
      new Range(MODE_ELEVEN_LOW_VOLTAGE, MODE_ELEVEN_HIGH_VOLTAGE),
      new Range(MODE_TWELVE_LOW_VOLTAGE, MODE_TWELVE_HIGH_VOLTAGE),
  };

  
  
  /** Creates a new AnalogSelectorSubsystem. */
  private final static AnalogSelectorSubsystem INSTANCE_ANALOG_SELECTOR = new AnalogSelectorSubsystem();
  private AnalogInput analogInput;


  public AnalogSelectorSubsystem() 
  {
    if(Config.ANALOG_SELECTOR_PORT != -1)
    {
      initializeSubsystem();
    }
    else
    {
      analogInput = null;
    }
  }

  public void initializeSubsystem()
  {
    analogInput = new AnalogInput(Config.ANALOG_SELECTOR_PORT);
  }

  public boolean isActive()
  {
    return analogInput != null;
  }

  public static AnalogSelectorSubsystem getInstance()
  {
    if(INSTANCE_ANALOG_SELECTOR.isActive() == true)
    {
      return INSTANCE_ANALOG_SELECTOR;
    }
    else
    {
      return null;
    }
  }
  public int getIndex() {

    final double voltage = analogInput.getAverageVoltage();
    // System.out.println("selector average value"+voltage);

    int index = 0;
    // Check each voltage range
    for (int i = 0; i < VOLTAGE_RANGES.length; i++) {
        // Check if the voltage is within the current voltage range
        if (VOLTAGE_RANGES[i].isWithin(voltage)) {
            index = i;
            break;
        }
    }
    return index - 1;
}

public double getVoltage() {
    double voltage = analogInput.getVoltage();
    return voltage;
}

public double getAverageVoltage() {
    double avgVoltage = analogInput.getAverageVoltage();
    return avgVoltage;
}

public static class Range {
    public final double min, max;

    public Range(double min, double max) {
        this.min = min;
        this.max = max;
    }

    /**
     * Determines if the number is within this range.
     *
     * @param number The number to be tested.
     * @return True if it's within range, false otherwise.
     */
    public boolean isWithin(final double number) {
        return min <= number && number < max;
    }
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    //for testing only
    // SmartDashboard.putNumber("AnalogSwitch", getIndex());

  }
}
