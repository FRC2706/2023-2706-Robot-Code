// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DiffTalonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceSwerve extends CommandBase {
  
  PigeonIMU m_pigeon = new PigeonIMU(Config.CANID.PIGEON);
  // Servo servoCam = new Servo(0);

  double balanceSpeedSlow = 0.24;
  double balanceSpeed = 0.35;

  /** Creates a new Balance. */
  public BalanceSwerve() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Find the reset method
    SwerveSubsystem.getInstance().resetPigeon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SwerveSubsystem.getInstance().getPitchVal()[1] <= 3) {
      SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow - (m_pigeon.getPitch()) / 15, 0, false, true);
  }
  else if (SwerveSubsystem.getInstance().getPitchVal()[1] < 10) {
      if (m_pigeon.getPitch() > 0) {
          SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
      }
      else if (m_pigeon.getPitch() < 0) {
          SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
      }
  }
  else {
      if (m_pigeon.getPitch() > 0) {
          while (m_pigeon.getPitch() > 10) {
              SwerveSubsystem.getInstance().drive(0, balanceSpeed, 0, false, true);
          }
          while (m_pigeon.getPitch() > 0) {
              SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
          }
          while (m_pigeon.getPitch() < 0) {
              SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
          }
      }
      else {
          while (m_pigeon.getPitch() < -10) {
            SwerveSubsystem.getInstance().drive(0, -balanceSpeed, 0, false, true);
          }
          while (m_pigeon.getPitch() < 0) {
            SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
          }
          while (m_pigeon.getPitch() > 0) {
            SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
          }
      }
  }
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
