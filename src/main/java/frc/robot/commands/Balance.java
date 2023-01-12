// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;

import frc.robot.subsystems.GyroBalance;

public class Balance extends CommandBase {
  
  PigeonIMU m_pigeon = new PigeonIMU(Config.CANID.PIGEON);
  Servo servoCam = new Servo(0);

  /** Creates a new Balance. */
  public Balance() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Find the reset method
    m_pigeon.getResetCount();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(Math.round(m_pigeon.getPitch()));
    servoCam.set(0.5 - m_pigeon.getPitch()/175);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
