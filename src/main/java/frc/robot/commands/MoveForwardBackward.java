// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveForwardBackward extends CommandBase {

  PIDController pid = new PIDController(0, 0, 0);

  /** Creates a new MoveForwardBackward. */
  public MoveForwardBackward() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveSubsystem.getInstance().drive(
      pid.calculate(0, 0), //tbd
      pid.calculate(0, 0), //tbd
      0, 
      true, 
      true
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /*if (!interrupted) {
      Command balCommand = new BalanceSwerve();
      Trigger.whileActiveOnce(balCommand, true);
    }*/
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
