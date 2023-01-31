// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateAngle extends CommandBase {

  PIDController pid = new PIDController(0, 0, 0); //pid to be tested
  double newYaw;
  double angle;

  //tolerance for rotation
  double tolerance = 0.5;

  /** Creates a new RotateAngle. */
  public RotateAngle(double _angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(SwerveSubsystem.getInstance());

    this.angle = _angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SwerveSubsystem.getInstance().resetPigeon();

    this.newYaw = this.angle + SwerveSubsystem.getInstance().getYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveSubsystem.getInstance().drive(
      0, 
      0, 
      pid.calculate(SwerveSubsystem.getInstance().getYaw(), this.newYaw), 
      false, 
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(SwerveSubsystem.getInstance().getYaw() - this.newYaw) <= tolerance) {
      return(true);
    }
    return false;
  }
}
