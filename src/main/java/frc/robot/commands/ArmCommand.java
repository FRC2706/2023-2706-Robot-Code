// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotcontainers.CompRobotContainer.RobotGamePieceState;

public class ArmCommand extends CommandBase {
  RobotGamePieceState m_robotState;
  /** Creates a new ArmCommand. */
  //position: top, middle, bottom
  //0: top
  //1: middle
  //2: bottom
  public ArmCommand(Supplier<RobotGamePieceState> robotState, int position) {
    //get the current robotState: from container
    m_robotState = robotState.get();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
