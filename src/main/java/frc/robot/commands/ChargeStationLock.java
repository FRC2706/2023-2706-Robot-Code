// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class ChargeStationLock extends CommandBase {
  /** Creates a new ChargeStationLock. */
  public ChargeStationLock() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState state1 = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState state2 = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    SwerveSubsystem.getInstance().setModuleStatesNoAntiJitter(new SwerveModuleState[]{state1, state2, state2, state1}, false);
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
