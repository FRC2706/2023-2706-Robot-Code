// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.config.Config;
import frc.robot.subsystems.DiffTalonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class BalanceSwerve extends CommandBase {

  // Servo servoCam = new Servo(0);

  double balanceSpeedSlow = 0.24;
  double balanceSpeed = 0.35;
  PIDController pid = new PIDController(0, 0, 0);
  double initialAngle = 0;

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
    initialAngle = SwerveSubsystem.getInstance().getPitchValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveSubsystem.getInstance().drive(
      0,
      pid.calculate(SwerveSubsystem.getInstance().getPitch(), 0),  //could be x or y
      0, 
      false, 
      true
    );



    if (SwerveSubsystem.getInstance().getPitchValue() >= initialAngle - 0.05 && SwerveSubsystem.getInstance().getPitchValue() <= initialAngle + 0.05) {
      SwerveSubsystem.getInstance().drive(0, 0, 0, false, true);
    }
    else if (SwerveSubsystem.getInstance().getPitchValue() <= initialAngle + 3) {
      SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow - (SwerveSubsystem.getInstance().getPitch()) / 15, 0, false, true);
  }
  else if (SwerveSubsystem.getInstance().getPitchValue() < initialAngle + 10) {
      if (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
          SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
      }
      else if (SwerveSubsystem.getInstance().getPitchValue() < initialAngle) {
          SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
      }
  }
  else {
      if (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
          while (SwerveSubsystem.getInstance().getPitchValue() > initialAngle + 10) {
              SwerveSubsystem.getInstance().drive(0, balanceSpeed, 0, false, true);
          }
          while (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
              SwerveSubsystem.getInstance().drive(0, balanceSpeedSlow, 0, false, true);
          }
          while (SwerveSubsystem.getInstance().getPitchValue() < initialAngle) {
              SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
          }
      }
      else {
          while (SwerveSubsystem.getInstance().getPitchValue() < initialAngle - 10) {
            SwerveSubsystem.getInstance().drive(0, -balanceSpeed, 0, false, true);
          }
          while (SwerveSubsystem.getInstance().getPitchValue() < initialAngle) {
            SwerveSubsystem.getInstance().drive(0, -balanceSpeedSlow, 0, false, true);
          }
          while (SwerveSubsystem.getInstance().getPitchValue() > initialAngle) {
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
